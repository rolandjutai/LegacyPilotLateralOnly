#!/usr/bin/env python3
import gc

import cereal.messaging as messaging
from cereal import car
from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.controls.lib.events import Events
from openpilot.selfdrive.legacy_monitoring.driver_monitor import DriverStatus


def dmonitoringd_thread(sm=None, pm=None):
  gc.disable()
  set_realtime_priority(2)

  if pm is None:
    pm = messaging.PubMaster(['driverMonitoringState'])

  if sm is None:
    sm = messaging.SubMaster(['driverState', 'liveCalibration', 'carState', 'controlsState', 'modelV2'], poll=['driverState'])

  is_rhd = Params().get_bool("IsRhdDetected")

  driver_status = DriverStatus(rhd=is_rhd)

  sm['liveCalibration'].calStatus = log.LiveCalibrationData.Status.invalid
  sm['liveCalibration'].rpyCalib = [0, 0, 0]
  sm['carState'].buttonEvents = []
  sm['carState'].standstill = True

  v_cruise_last = 0
  driver_engaged = False

  # 10Hz <- dmonitoringmodeld
  while True:
    sm.update()

    # Mock driverMonitoringState packet to simulate no issues
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "events": [],
      "faceDetected": True,  # Assume face is always detected
      "isDistracted": False,  # Always assume no distractions
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
      "isRHD": is_rhd,  # Keep legacy setting intact
    }
    pm.send('driverMonitoringState', dat)


def main(sm=None, pm=None):
  dmonitoringd_thread(sm, pm)


if __name__ == '__main__':
  main()
