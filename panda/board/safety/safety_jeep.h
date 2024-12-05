// lateral limits
const SteeringLimits JEEP_STEERING_LIMITS = {
  .max_steer = 300,
  .max_torque_error = 80,
  .max_rt_delta = 75,        // 2 max rate up * 100Hz LKA send rate * 250000 RT interval / 1000000 = 50 ; 50 * 1.5 for safety pad = 75
  .max_rt_interval = 250000,
  .max_rate_up = 2,
  .max_rate_down = 2,
  .driver_torque_allowance = 80,
  .driver_torque_factor = 2,
  .type = TorqueDriverLimited,
};

#define JEEP_ABS_1           0xEE // CRC-8/SAE-J1850
#define JEEP_ABS_2           0xFE // CRC-8/SAE-J1850
#define JEEP_ABS_3           0xFA // CRC-8/SAE-J1850 not regular bits
#define JEEP_ABS_6           0x101 // CRC-8/SAE-J1850
#define JEEP_ACC_1           0x5A2 // No counter and checksum
#define JEEP_ACC_2           0x1F2 // No counter and checksum
#define JEEP_ACC_3           0x2FA // CRC-8/SAE-J1850 not regular bits
#define JEEP_ACC_4           0x73C // No counter and checksum
#define JEEP_ENGINE_1        0xFC // CRC-8/SAE-J1850
#define JEEP_ENGINE_2        0x1F0 // No counter and checksum
#define JEEP_EPS_1           0xDE // CRC-8/SAE-J1850
#define JEEP_EPS_2           0x106 // CRC-8/SAE-J1850
#define JEEP_LKA_COMMAND     0x1F6 // CRC-8/SAE-J1850
#define JEEP_LKA_HUD_2       0x547 // No counter and checksum
#define JEEP_BCM_1           0x73E // No counter and checksum

// TODO: need to find a button message for cancel spam
const CanMsg JEEP_TX_MSGS[] = {{JEEP_LKA_COMMAND, 0, 4}, {JEEP_LKA_HUD_2, 0, 8}, {JEEP_ACC_1, 0, 8}};

<<<<<<< HEAD
bool cruise_engaged = false;

=======
>>>>>>> parent of 553737440 (try fix)
// TODO: need to find a message for driver gas
// TODO: re-check counter/checksum for ABS_3
// TODO: reenable checksums/counters on ABS_1 and EPS_3 once checksums are bruteforced
RxCheck jeep_rx_checks[] = {
  {.msg = {{JEEP_ABS_1, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_ABS_2, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_ABS_3, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_ABS_6, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_ACC_2, 1, 8, .check_checksum = false, .max_counter = 0U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_ACC_3, 1, 4, .check_checksum = false, .max_counter = 0U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_ACC_4, 1, 8, .check_checksum = false, .max_counter = 0U, .frequency = 1U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_ENGINE_1, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_ENGINE_2, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_EPS_1, 0, 6, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_EPS_2, 0, 7, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{JEEP_BCM_1, 0, 4, .check_checksum = false, .max_counter = 0U, .frequency = 4U}, { 0 }, { 0 }}},
};

uint8_t jeep_crc8_lut_j1850[256];  // Static lookup table for CRC8 SAE J1850

static uint32_t jeep_get_checksum(const CANPacket_t *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1U;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static uint8_t jeep_get_counter(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  if (addr == 0xFA) { //ABS_3 have different counter byte
    return (uint8_t)(((GET_BYTE(to_push, 4U) & 0x78) >> 3) & 0xFU);
  } else {
    int counter_byte = GET_LEN(to_push) - 2U;
    return (uint8_t)(GET_BYTE(to_push, counter_byte) & 0xFU);
  }
}

static uint32_t jeep_compute_crc(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);

  // CRC is in the last byte, poly is same as SAE J1850 but uses a different init value and output XOR
  // For some addresses it uses standard SAE J8150
  uint8_t crc = 0U;
  if (addr == 0x1F6 || addr == 0xEE || addr == 0xFE || addr == 0xFA || addr == 0xFC || addr == 0xDE || addr == 0x106 || addr == 0x101) {
    crc = 0xFFU;  
  }
  
  for (int i = 0; i < len - 1; i++) {
    crc ^= (uint8_t)GET_BYTE(to_push, i);
    crc = jeep_crc8_lut_j1850[crc];
  }

  // TODO: bruteforce final XORs for Panda relevant messages
  
  uint8_t final_xor = 0x0;
  if (addr == 0x1F6 || addr == 0xEE || addr == 0xFE || addr == 0xFA || addr == 0xFC || addr == 0xDE || addr == 0x106 || addr == 0x101) {
    final_xor = 0xFFU;  
  }

  return (uint8_t)(crc ^ final_xor);
}

static safety_config jeep_init(uint16_t param) {
  UNUSED(param);

  brake_pressed = false;
  gas_pressed = false;

  gen_crc_lookup_table_8(0x1D, jeep_crc8_lut_j1850);
  return BUILD_SAFETY_CFG(jeep_rx_checks, JEEP_TX_MSGS);
}

static void jeep_rx_hook(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  // Update in-motion state by sampling wheel speeds
  if ((GET_BUS(to_push) == 0U) && (addr == JEEP_ABS_1)) {
    // Thanks, FCA, for these 13 bit signals. Makes perfect sense. Great work.
    // Signals: ABS_1.WHEEL_SPEED_[FL,FR,RL,RR]
    int wheel_speed_fl = (GET_BYTE(to_push, 1) >> 3) | (GET_BYTE(to_push, 0) << 5);
    int wheel_speed_fr = (GET_BYTE(to_push, 3) >> 6) | (GET_BYTE(to_push, 2) << 2) | ((GET_BYTE(to_push, 1) & 0x7U) << 10);
    int wheel_speed_rl = (GET_BYTE(to_push, 4) >> 1) | ((GET_BYTE(to_push, 3) & 0x3FU) << 7);
    int wheel_speed_rr = (GET_BYTE(to_push, 6) >> 4) | (GET_BYTE(to_push, 5) << 4) | ((GET_BYTE(to_push, 4) & 0x1U) << 12);
    vehicle_moving = (wheel_speed_fl + wheel_speed_fr + wheel_speed_rl + wheel_speed_rr)*0.017/4 > 0;
  }

  if ((GET_BUS(to_push) == 0U) && (addr == JEEP_EPS_2)) {
    int torque_meas_new = ((GET_BYTE(to_push, 3) >> 5) | (GET_BYTE(to_push, 2) << 3)) - 1024U;
    update_sample(&torque_meas, torque_meas_new);
  }

  // driver torque
  if ((GET_BUS(to_push) == 0U) && (addr == JEEP_EPS_2)) {
    int torque_driver_new = ((GET_BYTE(to_push, 1) >> 4) | (GET_BYTE(to_push, 0) << 4)) * 0.5 - 1000U;
    update_sample(&torque_driver, torque_driver_new);
  }

  // TODO: find cruise button message

  if ((GET_BUS(to_push) == 0U) && (addr == JEEP_ENGINE_2)) {
    float gas_pedal = ((GET_BYTE(to_push, 1) >> 5) | ((GET_BYTE(to_push, 0) & 0x1FU) << 3)) * 0.4;
    gas_pressed = gas_pedal > 0;
  }  
    
  // Signal: ABS_3.BRAKE_PEDAL_SWITCH
  if ((GET_BUS(to_push) == 0U) && (addr == JEEP_ABS_3)) {
    brake_pressed = GET_BIT(to_push, 3);
  }
  
  if ((GET_BUS(to_push) == 1U) && (addr == JEEP_ACC_2)) {
    // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
    // Always exit controls on main switch off
    int acc_status = (GET_BYTE(to_push, 4) & 0x0FU);
    bool cruise_engaged = (acc_status == 6) || (acc_status == 7) || (acc_status == 8);
    acc_main_on = cruise_engaged;

    pcm_cruise_check(cruise_engaged);

    if (!acc_main_on) {
      controls_allowed = false;
    }
  }
  
  // If steering controls messages are received on the destination bus, it's an indication
  // that the relay might be malfunctioning

  generic_rx_checks((GET_BUS(to_push) == 0U) && ((addr == JEEP_LKA_COMMAND) || (addr == JEEP_LKA_HUD_2) || (addr == JEEP_ACC_1)));
}

static bool jeep_tx_hook(const CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);
  bool tx = true;

  // STEERING
  if (addr == JEEP_LKA_COMMAND) {
    int desired_torque = ((GET_BYTE(to_send, 1) >> 5) | (GET_BYTE(to_send, 0) << 3));
    desired_torque -= 1024;
    bool steer_req = GET_BIT(to_send, 12U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, JEEP_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // TODO: sanity check cancel spam, once a button message is found

  // FIXME: don't actually run any checks during early testing
  //tx = true;

  return tx;
}

static int jeep_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      bus_fwd = 2;
      break;
    case 2:
      if ((addr == JEEP_LKA_COMMAND) || (addr == JEEP_LKA_HUD_2) || (addr == JEEP_ACC_1)) {
        bus_fwd = -1;
      } else {
        bus_fwd = 0;
      }
      break;
    default:
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

const safety_hooks jeep_hooks = {
  .init = jeep_init,
  .rx = jeep_rx_hook,
  .tx = jeep_tx_hook,
  .fwd = jeep_fwd_hook,
  .get_counter = jeep_get_counter,
  .get_checksum = jeep_get_checksum,
  .compute_checksum = jeep_compute_crc,
};
