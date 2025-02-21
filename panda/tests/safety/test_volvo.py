#!/usr/bin/env python3
import unittest
from panda import Panda
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common
from panda.tests.safety.common import CANPackerPanda


class TestVolvoSafety(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest):
  TX_MSGS = [[0x127, 0], [0x246, 2], [0x262, 0], [0x270, 0]]
  STANDSTILL_THRESHOLD = 0
  GAS_PRESSED_THRESHOLD = 10
  RELAY_MALFUNCTION_ADDRS = {0: (0x127,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x246], 2: [0x127, 0x262, 0x270]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  EPS_BUS = 0
  CRUISE_BUS = 2

  # Angle control limits
  DEG_TO_CAN = 100

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [5., .8, .15]  # windup limit
  ANGLE_RATE_DOWN = [5., 3.5, .4]  # unwind limit

  def setUp(self):
    self.packer = CANPackerPanda("volvo_v60_2015_pt")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_VOLVO, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float):
    values = {"SteeringAngleServo": angle}
    return self.packer.make_can_msg_panda("PSCM1", 0, values)

  def _angle_meas_msg(self, angle: float):
    values = {"SteeringAngleServo": angle}
    return self.packer.make_can_msg_panda("STEER_ANGLE_SENSOR", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCStatus": enable}
    return self.packer.make_can_msg_panda("FSM0", 0, values)

  def _speed_msg(self, speed):
    values = {"VehicleSpeed": speed / 3.6}
    return self.packer.make_can_msg_panda("VehicleSpeed1", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePedal": 1 if brake == 2 else 0}
    return self.packer.make_can_msg_panda("Brake_Info", 0, values)

  def _user_gas_msg(self, gas_pressed):
    values = {"AccPedal": 1 if gas_pressed > 10 else 0}
    return self.packer.make_can_msg_panda("AccPedal", 0, values)

if __name__ == "__main__":
  unittest.main()
