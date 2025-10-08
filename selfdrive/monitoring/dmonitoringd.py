#!/usr/bin/env python3
import gc

import cereal.messaging as messaging
from cereal import car
from cereal import log
from openpilot.common.params import Params, put_bool_nonblocking
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.controls.lib.events import Events
from openpilot.selfdrive.monitoring.driver_monitor import DriverStatus


def dmonitoringd_thread(sm=None, pm=None):
  gc.disable()
  set_realtime_priority(2)

  if pm is None:
    pm = messaging.PubMaster(['driverMonitoringState'])

  if sm is None:
    sm = messaging.SubMaster(['driverStateV2', 'liveCalibration', 'carState', 'controlsState', 'modelV2'], poll=['driverStateV2'])

  driver_status = DriverStatus(rhd_saved=Params().get_bool("IsRhdDetected"))

  sm['liveCalibration'].calStatus = log.LiveCalibrationData.Status.invalid
  sm['liveCalibration'].rpyCalib = [0, 0, 0]
  sm['carState'].buttonEvents = []
  sm['carState'].standstill = True

  v_cruise_last = 0
  driver_engaged = False

  # 10Hz <- dmonitoringmodeld
  # Skip driver monitoring updates
  while True:
    sm.update()

    # Mock driverMonitoringState packet to simulate no issues
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "events": [],
      "faceDetected": True,  # Assume face is always detected
      "isDistracted": False,  # Assume driver is not distracted
      "distractedType": 0,
      "awarenessStatus": 1.0,
      "posePitchOffset": 0.0,
      "posePitchValidCount": 0,
      "poseYawOffset": 0.0,
      "poseYawValidCount": 0,
      "stepChange": 0.0,
      "awarenessActive": 1.0,
      "awarenessPassive": 1.0,
      "isLowStd": True,
      "hiStdCount": 0,
      "isActiveMode": False,  # Disable active monitoring
      "isRHD": False
    }
    pm.send('driverMonitoringState', dat)

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and
     driver_status.wheelpos_learner.filtered_stat.n > driver_status.settings._WHEELPOS_FILTER_MIN_COUNT and
     driver_status.wheel_on_right == (driver_status.wheelpos_learner.filtered_stat.M > driver_status.settings._WHEELPOS_THRESHOLD)):
      put_bool_nonblocking("IsRhdDetected", driver_status.wheel_on_right)

def main(sm=None, pm=None):
  dmonitoringd_thread(sm, pm)


if __name__ == '__main__':
  main()
