from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.values import CAR

#ButtonType = car.CarState.ButtonEvent.Type
#EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate: CAR, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "volvo"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.volvo)]
    # ret.dashcamOnly = True
    ret.radarUnavailable = True

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.8

    return ret
