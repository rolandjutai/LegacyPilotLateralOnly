from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, common_fault_avoidance
from openpilot.selfdrive.car.hyundai import hyundaicanfd, hyundaican
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR
from types import SimpleNamespace
from collections import deque
import time

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning

from collections import deque
import time

class SmartCruiseController:
  def __init__(self):
    self.desired_speed = None
    self.last_target_speed = None
    self.cancel_active = False
    self.cancel_time = 0.0
    self.last_button_time = 0.0
    self.speed_error_buffer = deque(maxlen=50)  # ~1s at 50 Hz
    self.auto_resume_guard = 1.5
    self.last_resume_time = 0.0
    self.auto_cancel_flag = False

  def update(self, clu_speed_mps, current_cruise_speed_mps, v_ego_mps,
             lead, curvature, stop_prob, gas_cmd, accel_ego):

    now = time.monotonic()
    can_cmds = []

    # Convert to kph for heuristic thresholds
    clu_kph = clu_speed_mps * CV.MS_TO_KPH
    cruise_kph = current_cruise_speed_mps * CV.MS_TO_KPH
    v_kph = v_ego_mps * CV.MS_TO_KPH

    # --- Base desired speed (m/s) ---
    target_speed_mps = current_cruise_speed_mps

    # Lead vehicle adjustment
    if getattr(lead, 'modelProb', 0.0) > 0.5 and getattr(lead, 'dRel', 1e9) > 0:
      time_gap = lead.dRel / max(v_ego_mps, 0.1)
      if time_gap < 1.2:
        target_speed_mps = min(target_speed_mps, v_ego_mps + lead.vRel - 5.0)  # vRel is m/s

    # Curve adjustment (curvature in 1/m, thresholds tuned empirically)
    if curvature:
      curv_max = max(abs(c) for c in curvature[len(curvature)//2:])
      if (v_kph < 80.0 and curv_max > 0.02) or (v_kph >= 80.0 and curv_max > 0.03):
        curve_limit_kph = max(30.0, v_kph * (0.8 / (curv_max * 100.0)))
        target_speed_mps = min(target_speed_mps, curve_limit_kph * CV.KPH_TO_MS)

    # Stop line adjustment
    if stop_prob > 0.7 and v_kph < 45.0:
      target_speed_mps = 0.0

    # Hill bias correction (compare bias in kph)
    self.speed_error_buffer.append(clu_speed_mps - current_cruise_speed_mps)
    if len(self.speed_error_buffer) == self.speed_error_buffer.maxlen:
      avg_bias_kph = (sum(self.speed_error_buffer) / len(self.speed_error_buffer)) * CV.MS_TO_KPH
      if avg_bias_kph < -2.0:   # undershooting uphill
        target_speed_mps += 2.0 * CV.KPH_TO_MS
      elif avg_bias_kph > 2.0:  # overshooting downhill
        target_speed_mps -= 2.0 * CV.KPH_TO_MS

    # Smoothing (in m/s)
    smoothed = target_speed_mps if self.last_target_speed is None else 0.7 * self.last_target_speed + 0.3 * target_speed_mps
    self.last_target_speed = smoothed
    self.desired_speed = smoothed

    # --- Decide button actions in 1 kph quanta ---
    diff_kph = (self.desired_speed - current_cruise_speed_mps) * CV.MS_TO_KPH

    # Emergency auto-cancel
    urgent = (diff_kph < -10.0) or (getattr(lead, 'modelProb', 0.0) > 0.5 and getattr(lead, 'dRel', 1e9) < 6.0 and getattr(lead, 'vRel', 0.0) < -5.0)
    if urgent and not self.cancel_active:
      can_cmds.append(("CANCEL", True))
      self.cancel_active = True
      self.cancel_time = now
      self.auto_cancel_flag = True
      return can_cmds

    # While canceled, auto-resume when near target
    if self.cancel_active:
      if v_ego_mps <= self.desired_speed + 1.0 and (now - self.cancel_time) > self.auto_resume_guard:
        can_cmds.append(("SET", True))
        self.cancel_active = False
        self.auto_cancel_flag = False
      return can_cmds

    # Normal fine-tuning: 1 press per kph, max 4 every ~0.3s
    if abs(diff_kph) >= 1.0:
      if now - self.last_button_time > 0.3:
        presses = min(4, int(abs(diff_kph)))
        button = "RES_ACCEL" if diff_kph > 0 else "SET_DECEL"
        for _ in range(presses):
          can_cmds.append((button, False))
        self.last_button_time = now

    return can_cmds


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.frame = 0

    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    # Instantiate SmartCruise controller
    self.smartCruise = SmartCruiseController()
    self.op_long_target_speed = 0.0

  def update(self, CC, CS, now_nanos, lead_one, curvatures, stopline_prob):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_steer = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends = []

      # --- SmartCruise button spam (classic CAN only; independent of OP longitudinal) ---
    if getattr(CS, 'smartcruise_active', False) and not self.CP.openpilotLongitudinalControl and (self.CP.carFingerprint not in CANFD_CAR):
      # Safe defaults
      lead = lead_one if lead_one is not None else SimpleNamespace(modelProb=0.0, dRel=1e9, vRel=0.0)
      curv = list(curvatures) if curvatures else []
      stop_prob = float(stopline_prob) if stopline_prob is not None else 0.0
  
      clu_speed = getattr(CS.out, 'vEgoCluster', CS.out.vEgo)                 # m/s
      current_cruise_speed = getattr(CS.out.cruiseState, 'speed', CS.out.vEgo)  # m/s
      v_ego = CS.out.vEgo
      gas = CS.out.gas
      a_ego = CS.out.aEgo
  
      cmds = self.smartCruise.update(
        clu_speed,
        current_cruise_speed,
        v_ego,
        lead,
        curv,
        stop_prob,
        gas,
        a_ego,
      )
  
      for cmd, auto_cancel in cmds:
        if cmd == "RES_ACCEL":
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
        elif cmd == "SET_DECEL":
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
        elif cmd == "CANCEL":
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP.carFingerprint))
          CS.auto_cancel = auto_cancel  # let interface consume next cycle
        elif cmd == "SET":
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
    
    # If lateral not active, do not send any steering/EPS/HUD traffic.
    # SmartCruise button pulses (CLU11) above are still allowed to go out.
    # If lateral not active, still send LKAS11 keepalive with zero torque to avoid EPS/LKA faults.
    # SmartCruise button pulses (CLU11) above are still allowed to go out.
    if not CC.latActive:
      if self.CP.carFingerprint not in CANFD_CAR:
        # Minimal, conservative HUD fields to avoid any LKAS/LFA activation hints
        can_sends.append(hyundaican.create_lkas11(
          self.packer, self.frame, self.car_fingerprint,
          0,            # apply_steer
          False,        # apply_steer_req
          False,        # torque_fault
          CS.lkas11,
          False,        # sys_warning
          1,            # sys_state: default/no lines
          False,        # enabled flag for HUD
          False, False, # leftLaneVisible, rightLaneVisible
          0, 0          # left_lane_warning, right_lane_warning
        ))
      else:
        # If you test on CAN-FD cars later, send a zero-torque steering message instead of nothing:
        # can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, False, False, 0))
        pass

      new_actuators = actuators.copy()
      new_actuators.steer = 0.0
      new_actuators.steerOutputCan = 0
      new_actuators.accel = accel
      self.frame += 1
      return new_actuators, can_sends
    
    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append([0x7b1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", self.CAN.ECAN])

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_steer))

      # prevent LFA from activating on HDA2 by sending "no lane lines detected" to ADAS ECU
      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.hda2_lfa_block_msg,
                                                          self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

      # blinkers
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units))
          self.accel_last = accel
      else:
        # button presses
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, apply_steer_req,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning))

      if not self.CP.openpilotLongitudinalControl:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        # TODO: unclear if this is needed
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                        hud_control.leadVisible, set_speed_in_units, stopping,
                                                        CC.cruiseControl.override, use_fca))

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.extend(hyundaican.create_acc_opt(self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def create_button_messages(self, CC: car.CarControl, CS: car.CarState, use_clu11: bool):
    # Button presses handled entirely by SmartCruiseController (classic CAN) or not used
    return []
