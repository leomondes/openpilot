
def create_steering_control(packer, bus, apply_steer, lkas_active):
  values = {
    "LKA_TORQUE": apply_steer,
    "LKA_ENABLED": 1 if lkas_active else 0,
  }

  return packer.make_can_msg("LKA_COMMAND", bus, values)

def create_lka_hud_2_control(packer, bus, lkas_active, auto_high_beam):
  values = {
    "LKA_ACTIVE": 6 if lkas_active else 1,
    "NEW_SIGNAL_1": 1,
    "HIGH_BEAM_ALLOWED": auto_high_beam,
  }

  return packer.make_can_msg("LKA_HUD_2", bus, values)

def create_acc_1_control(packer, bus, apply_steer):
  values = {
    "LKA_CHECK": 1 if apply_steer != 0 else 0,
  }

  return packer.make_can_msg("ACC_1", bus, values)

# LKA_ACTIVE
# 6 (0110) = green car with both white lines
# 10 (1010) or 12 (1100) yellow on right, positive torque to turn left
# 8 (1000) or 11 (1011) yellow on left, negative torque to turn right
