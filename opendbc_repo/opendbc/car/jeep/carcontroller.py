from opendbc.can.packer import CANPacker
from opendbc.car import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase

from opendbc.car.jeep import jeepcan
from opendbc.car.jeep.values import CANBUS, CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_name[0])

    self.apply_steer_last = 0
    self.frame = 0
    
  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if CS.out.vEgo > self.CP.minSteerSpeed and CS.out.cruiseState.available:
      lkas_active = True
    else:
      lkas_active = False

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:

      if CC.latActive and lkas_active:
        new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
      else:
        apply_steer = 0

      self.apply_steer_last = apply_steer
      can_sends.append(jeepcan.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, lkas_active))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.HUD_2_STEP == 0:
      can_sends.append(jeepcan.create_lka_hud_2_control(self.packer_pt, CANBUS.pt, lkas_active, CS.auto_high_beam))
    
    if self.frame % self.CCP.ACC_1_STEP == 0:
      can_sends.append(jeepcan.create_acc_1_control(self.packer_pt, CANBUS.pt, apply_steer))

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.frame += 1
    return new_actuators, can_sends
