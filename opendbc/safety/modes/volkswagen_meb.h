#pragma once

#include "opendbc/safety/modes/volkswagen_meb_base.h"

static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay = true}, {MSG_GRA_ACC_01, 0, 8, .check_relay = false},
                                                       {MSG_EA_01, 0, 8, .check_relay = false}, {MSG_EA_02, 0, 8, .check_relay = true},
                                                       {MSG_KLR_01, 0, 8, .check_relay = false}, {MSG_KLR_01, 2, 8, .check_relay = false},
                                                       {MSG_GRA_ACC_01, 2, 8, .check_relay = false}, {MSG_LDW_02, 0, 8, .check_relay = true}};
  
  static const CanMsg VOLKSWAGEN_MEB_LONG_TX_MSGS[] = {{MSG_MEB_ACC_01, 0, 48, .check_relay = true}, {MSG_ACC_18, 0, 32, .check_relay = true}, {MSG_HCA_03, 0, 24, .check_relay = true},
                                                       {MSG_EA_01, 0, 8, .check_relay = false}, {MSG_EA_02, 0, 8, .check_relay = true},
                                                       {MSG_KLR_01, 0, 8, .check_relay = false}, {MSG_KLR_01, 2, 8, .check_relay = false},
                                                       {MSG_LDW_02, 0, 8, .check_relay = true}, {MSG_TA_01, 0, 8, .check_relay = true}};

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_LH_EPS_03, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_Motor_51, 0, 32, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01, 0, 32, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESC_51, 0, 48, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_Motor_54, 0, 32, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  UNUSED(param);

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#endif
  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_STOCK_TX_MSGS);
}

const safety_hooks volkswagen_meb_hooks = {
  .init = volkswagen_meb_init,
  .rx = volkswagen_meb_rx_hook,
  .tx = volkswagen_meb_tx_hook,
  .get_counter = volkswagen_meb_get_counter,
  .get_checksum = volkswagen_meb_get_checksum,
  .compute_checksum = volkswagen_meb_compute_crc,
};
