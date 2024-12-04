from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.jeep.values import CAR


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate: CAR, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "jeep"
    ret.radarUnavailable = True

    # Set global parameters

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.jeep)]

    # Global lateral tuning defaults, can be overridden per-vehicle

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.2 # default 0.1
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # Global longitudinal tuning defaults, can be overridden per-vehicle

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    return ret

  # returns a car.CarState
  def _update(self):
    #ret = self.CS.update(self.cp, self.cp_cam, self.cp_body)

    return ret

