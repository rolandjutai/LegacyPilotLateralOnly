from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, common_fault_avoidance
from openpilot.selfdrive.car.hyundai import hyundaicanfd, hyundaican
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

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
  """
  An add-on layer that emulates Vision-based Adaptive Cruise
  by spamming cruise buttons (SET/RES) or issuing CANCEL/SET
  to accelerate slowdown when needed.

  - Reads vision lead data (from radarState.leadOne)
  - Reads curvature (from lateralPlan.curvatures)
  - Outputs a list of (button presses) or CANCEL/RESUME
  """
  def __init__(self):
    self.desired_speed = None
    self.last_target_speed = None
    self.cancel_active = False
    self.cancel_time = 0.0
    self.last_button_time = 0.0
    self.speed_error_buffer = deque(maxlen=50)  # 1s averaging at 50Hz
    self.auto_resume_guard = 1.5
    self.last_resume_time = 0.0
    self.auto_cancel_flag = False   # distinguish driver cancel vs auto cancel

  def update(self, clu_speed, current_cruise_speed, v_ego,
             lead, curvature, stop_prob, throttle_cmd, accel_ego):

    now = time.monotonic()
    can_cmds = []

    # --- Base desired speed ---
    target_speed = current_cruise_speed

    # Lead vehicle adjustment
    if lead.modelProb > 0.5 and lead.dRel > 0:
      time_gap = lead.dRel / max(v_ego, 0.1)
      if time_gap < 1.2:
        target_speed = min(target_speed, v_ego + lead.vRel - 5)

    # Curve adjustment (only for sharp curves)
    if curvature:
      curv_max = max(abs(c) for c in curvature[len(curvature)//2:])
      if (v_ego < 80 and curv_max > 0.02) or (v_ego >= 80 and curv_max > 0.03):
        curve_limit = max(30.0, v_ego * (0.8 / (curv_max*100)))
        target_speed = min(target_speed, curve_limit)

    # Stop line adjustment
    if stop_prob > 0.7 and v_ego < 45:
      target_speed = 0.0

    # Hill bias correction
    self.speed_error_buffer.append(clu_speed - current_cruise_speed)
    if len(self.speed_error_buffer) == self.speed_error_buffer.maxlen:
      avg_bias = sum(self.speed_error_buffer)/len(self.speed_error_buffer)
      if avg_bias < -2.0:   # undershooting uphill
        target_speed += 2.0
      elif avg_bias > 2.0:  # overshooting downhill
        target_speed -= 2.0

    # Smoothing
    if self.last_target_speed is None:
      smoothed = target_speed
    else:
      smoothed = 0.7*self.last_target_speed + 0.3*target_speed

    self.last_target_speed = smoothed
    self.desired_speed = smoothed

    # --- Decide button actions ---
    diff = self.desired_speed - current_cruise_speed

    # Emergency auto-cancel
    urgent = (diff < -10) or (lead.modelProb > 0.5 and lead.dRel < 6 and lead.vRel < -5)
    if urgent and not self.cancel_active:
      can_cmds.append(("CANCEL", True))   # (command, auto_cancel_flag)
      self.cancel_active = True
      self.cancel_time = now
      self.auto_cancel_flag = True
      return can_cmds

    # While canceled
    if self.cancel_active:
      if v_ego <= self.desired_speed+1.0 and (now - self.cancel_time) > self.auto_resume_guard:
        can_cmds.append(("SET", True))
        self.cancel_active = False
        self.auto_cancel_flag = False
      return can_cmds

    # Normal fine-tuning
    if abs(diff) >= 1.0:
      if now - self.last_button_time > 0.3:  # 3 Hz max
        presses = min(4, int(abs(diff)))
        button = "RES_ACCEL" if diff > 0 else "SET_DECEL"
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

    # NEW: track OP long target speed
    self.op_long_target_speed = 0.0

    #Instantiate SmartCruiseController once - rest is done in update
    self.smartCruise = SmartCruiseController()   # instantiate once

  def update(self, CC, CS, now_nanos, lead_one, curvatures, stopline_prob):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE,
                                                                       CC.latActive,
                                                                       self.angle_limit_counter,
                                                                       MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)
    if not CC.latActive:
      apply_steer = 0

    torque_fault = CC.latActive and not apply_steer_req
    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(
        CC.enabled, self.car_fingerprint, hud_control)

    can_sends = []

    # We instantiated self.smartCruise = SmartCruiseController() in CarController.__init__(). Now each cycle we do:
    cmds = self.smartCruise.update(CS.clu_speed,
                               CS.current_cruise_speed,
                               CS.out.vEgo,
                               lead_one,
                               curvatures,
                               stopline_prob,
                               CS.out.throttle,
                               CS.out.aEgo)

    for cmd, auto_cancel in cmds:
      if cmd == "RES_ACCEL":
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame,
                                                 CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
      elif cmd == "SET_DECEL":
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame,
                                                 CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
      elif cmd == "CANCEL":
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame,
                                                 CS.clu11, Buttons.CANCEL, self.CP.carFingerprint))
        CS.auto_cancel = auto_cancel  # mark in CarState for Interface
      elif cmd == "SET":
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame,
                                                 CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))

    # --- Handle resumeRequired event (driver pressed RES while gas override) ---
    if any(e.name == car.CarEvent.EventName.resumeRequired for e in CC.events):
      self.op_long_target_speed = CS.out.vEgo

    # *** common hyundai stuff ***

    # tester present
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append([0x7b1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", self.CAN.ECAN])

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN,
                                                             CC.enabled, apply_steer_req, apply_steer))

      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN,
                                                          CS.hda2_lfa_block_msg,
                                                          self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING))

      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame,
                                                           CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled,
                                                           self.accel_last, accel, stopping,
                                                           CC.cruiseControl.override, set_speed_in_units))
          self.accel_last = accel
      else:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer,
                                                apply_steer_req, torque_fault, CS.lkas11,
                                                sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning))

      # always allow OP emulated long via button spam
      can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                        hud_control.leadVisible, set_speed_in_units, stopping,
                                                        CC.cruiseControl.override, use_fca))

      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.extend(hyundaican.create_acc_opt(self.packer))

      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def create_button_messages(self, CC, CS, use_clu11: bool):
    # Button presses handled entirely by SmartCruiseController
    return []
