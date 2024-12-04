import numpy as np
from cereal import car
from openpilot.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.car.jeep.values import DBC, CANBUS, CarControllerParams
from opendbc.car import Bus


GearShifter = car.CarState.GearShifter
STANDSTILL_THRESHOLD = 0

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.frame = 0
    self.CCP = CarControllerParams(CP)

  def update(self, pt_cp, cam_cp, cp_body):
    cp = can_parsers[Bus.pt]
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_FL"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_FR"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_RL"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_RR"],
      unit=1.0
    )

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    #ret.vEgoRaw = pt_cp.vl["ABS_6"]["VEHICLE_SPEED"] # speed from HUD
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < STANDSTILL_THRESHOLD

    ret.steeringAngleDeg = pt_cp.vl["EPS_1"]["STEERING_ANGLE"]
    ret.steeringRateDeg = pt_cp.vl["EPS_1"]["STEERING_RATE"]
    ret.steeringTorque = pt_cp.vl["EPS_2"]["DRIVER_TORQUE"]
    ret.steeringTorqueEps = pt_cp.vl["EPS_2"]["EPS_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > 80
    ret.yawRate = pt_cp.vl["ABS_2"]["YAW_RATE"]
    ret.steerFaultPermanent = bool(pt_cp.vl["EPS_2"]["LKA_FAULT"])

    # TODO: unsure if this is accel pedal or engine throttle
    #ret.gas = pt_cp.vl["ENGINE_1"]["ACCEL_PEDAL"]
    ret.gas = pt_cp.vl["ENGINE_2"]["ACCEL_PEDAL_FOOT"]
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["ABS_4"]["BRAKE_PRESSURE"]
    ret.brakePressed = bool(pt_cp.vl["ABS_3"]["BRAKE_PEDAL_SWITCH"])
    ret.parkingBrake = bool(pt_cp.vl["ENGINE_3"]["HANDBRAKE"]) 

    if pt_cp.vl["ENGINE_3"]["GEAR"] == 1:
      ret.gearShifter = GearShifter.park
    elif pt_cp.vl["ENGINE_3"]["GEAR"] == 2:
      ret.gearShifter = GearShifter.reverse
    elif pt_cp.vl["ENGINE_3"]["GEAR"] == 3:
      ret.gearShifter = GearShifter.neutral
    else:
      ret.gearShifter = GearShifter.drive

    #if bool(pt_cp.vl["ENGINE_1"]["REVERSE"]):
    #  ret.gearShifter = GearShifter.reverse
    #else:
    #  ret.gearShifter = GearShifter.drive

    ret.cruiseState.available = bool(cp_body.vl["ACC_4"]["ACC_AVAILABLE"])
    ret.cruiseState.enabled = cp_body.vl["ACC_2"]["ACC_ACTIVE"] in (6, 7, 8) and pt_cp.vl["ENGINE_3"]["GEAR"] > 1 
    # add check speed/gear because when turn off car for a brief moment ACC_ACTIVE = 7
    ret.cruiseState.speed = cp_body.vl["ACC_4"]["ACC_SPEED"] * CV.KPH_TO_MS

    self.auto_high_beam = bool(cam_cp.vl["LKA_HUD_2"]["HIGH_BEAM_ALLOWED"])

    ret.leftBlinker = bool(pt_cp.vl["BCM_1"]["LEFT_TURN_STALK"])
    ret.rightBlinker = bool(pt_cp.vl["BCM_1"]["RIGHT_TURN_STALK"])
    # ret.buttonEvents = TODO
    # ret.espDisabled = TODO

    self.frame += 1
    return ret


  @staticmethod
  def get_can_parser(CP):
    messages = [
      # sig_address, frequency
      ("ABS_1", 100),
      ("ABS_2", 100),
      ("ABS_3", 100),
      ("ABS_4", 50),
      ("ABS_6", 100),
      ("ENGINE_1", 100),
      ("ENGINE_2", 50),
      ("ENGINE_3", 1),
      ("EPS_1", 100),
      ("EPS_2", 100),
      ("BCM_1", 4),  # 4Hz plus triggered updates
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("LKA_HUD_2", 4),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }

  @staticmethod
  def get_body_can_parser(CP):
    messages = [
      # sig_address, frequency
      ("ACC_2", 50), #from cabana
      ("ACC_3", 50),
      ("ACC_4", 1), #from cabana
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
    