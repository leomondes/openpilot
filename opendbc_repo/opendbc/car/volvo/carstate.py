from cereal import car
from opendbc.can.parser import CANParser
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.volvo.values import CarControllerParams, DBC


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.cruiseState_enabled_prev = False
    self.eps_torque_timer = 0

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # car speed
    ret.vEgoRaw = cp.vl["VehicleSpeed1"]["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # gas pedal
    ret.gas = cp.vl["AccPedal"]["AccPedal"] / 100.0
    ret.gasPressed = cp.vl["AccPedal"]["AccPedal"] > 10  # compare the int to not mismatch panda

    # brake pedal
    ret.brake = cp.vl["BrakePedal"]["BrakePressure"]
    ret.brakePressed = cp.vl["Brake_Info"]["BrakePedal"] == 2

    # steering
    ret.steeringAngleDeg = cp.vl["PSCM1"]["SteeringAngleServo"]
    ret.steeringTorqueEps = cp.vl["PSCM1"]["LKATorque"]
    ret.steeringPressed = False  # TODO

    # cruise state
    ret.cruiseState.speed = cp.vl["ACC_Speed"]["ACC_Speed"] * CV.KPH_TO_MS
    ret.cruiseState.available = cp_cam.vl["FSM0"]["ACCStatus"] in (2, 6, 7)
    ret.cruiseState.enabled = cp_cam.vl["FSM0"]["ACCStatus"] in (6, 7)
    ret.cruiseState.standstill = cp_cam.vl["FSM3"]["ACC_Standstill"] == 1
    ret.cruiseState.nonAdaptive = False  # TODO
    ret.accFaulted = False
    self.acc_distance = cp_cam.vl["FSM1"]["ACC_Distance"]

    # Check if servo stops responding when ACC is active
    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
      # Reset counter on entry
      if not self.cruiseState_enabled_prev:
        self.eps_torque_timer = 0

      # Count up when no torque from servo detected
      if ret.steeringTorqueEps == 0:
        self.eps_torque_timer += 1
      else:
        self.eps_torque_timer = 0

      # Set fault if above threshold
      ret.steerFaultTemporary = self.eps_torque_timer >= CarControllerParams.STEER_TIMEOUT

    self.cruiseState_enabled_prev = ret.cruiseState.enabled

    # gear
    ret.gearShifter = car.CarState.GearShifter.drive  # TODO

    # safety
    ret.stockFcw = False  # TODO
    ret.stockAeb = False

    # button presses
    ret.leftBlinker = cp.vl["MiscCarInfo"]["TurnSignal"] == 1
    ret.rightBlinker = cp.vl["MiscCarInfo"]["TurnSignal"] == 3

    # lock info
    ret.doorOpen = not all([cp.vl["Doors"]["DriverDoorClosed"], cp.vl["Doors"]["PassengerDoorClosed"]])
    ret.seatbeltUnlatched = False  # TODO

    # Store info from servo message PSCM1
    # FSM (camera) checks if LKAActive & LKATorque active when not requested
    self.pscm_stock_values = cp.vl["PSCM1"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      # msg, freq
      ("VehicleSpeed1", 50),
      ("AccPedal", 100),
      ("BrakePedal", 50),
      ("Brake_Info", 50),
      ("PSCM1", 50),
      ("ACC_Speed", 50),
      ("MiscCarInfo", 25),
      ("Doors", 20),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      # msg, freq
      ("FSM0", 100),
      ("FSM1", 50),
      ("FSM3", 50),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)
