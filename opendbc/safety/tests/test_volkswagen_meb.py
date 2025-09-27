#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags

MAX_ACCEL = 2.0
MIN_ACCEL = -3.5
INACTIVE_ACCEL = 3.01
ACCEL_OVERRIDE = 0

# MEB message IDs
MSG_ESC_51        = 0xFC
MSG_LH_EPS_03     = 0x9F
MSG_QFK_01        = 0x13D
MSG_Motor_54      = 0x14C
MSG_Motor_51      = 0x10B
MSG_ACC_18        = 0x14D
MSG_MEB_ACC_01    = 0x300
MSG_HCA_03        = 0x303
MSG_GRA_ACC_01    = 0x12B
MSG_LDW_02        = 0x397
MSG_MOTOR_14      = 0x3BE
MSG_TA_01         = 0x26B
MSG_KLR_01        = 0x25D
MSG_EA_01         = 0x1A4
MSG_EA_02         = 0x1F0


class TestVolkswagenMebSafetyBase(common.PandaCarSafetyTest, common.SteerCurvatureSafetyTest):
  # === limits ===
  MAX_CURVATURE = 29105
  MAX_RT_DELTA = 75
  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 10

  CURVATURE_TO_CAN = 149253.7313
  INACTIVE_CURVATURE_IS_ZERO = True
  ROLL_TO_CAN = 10000.0

  # Steering driver torque
  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 3

  # === helper messages ===
  def _speed_msg(self, speed):
    values = {
      "HL_Radgeschw": speed,
      "HR_Radgeschw": speed,
      "VL_Radgeschw": speed,
      "VR_Radgeschw": speed,
    }
    return self.packer.make_can_msg_panda("ESC_51", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_panda("LH_EPS_03", 0, values)

  def _curvature_meas_msg(self, curvature):
    values = {"Curvature": abs(curvature), "Curvature_VZ": curvature < 0}
    return self.packer.make_can_msg_panda("QFK_01", 0, values)

  def _curvature_cmd_msg(self, curvature, steer_req=1, power=50):
    values = {
      "Curvature": abs(curvature),
      "Curvature_VZ": curvature < 0,
      "RequestStatus": 4 if steer_req else 0,
      "Power": power,
    }
    return self.packer.make_can_msg_panda("HCA_03", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Pressure": gas}
    return self.packer.make_can_msg_panda("Motor_54", 0, values)

  def _user_brake_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_panda("Motor_14", 0, values)

  def _tsk_status_msg(self, enable, main_switch=True):
    acc_status = 3 if enable else (2 if main_switch else 0)
    values = {"TSK_Status": acc_status}
    return self.packer.make_can_msg_panda("Motor_51", 0, values)

  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_panda("GRA_ACC_01", 0, values)

  def _accel_msg(self, accel):
    values = {"ACC_Sollbeschleunigung_02": accel}
    return self.packer.make_can_msg_panda("ACC_18", 0, values)

  # === basic tests ===
  def test_torque_measurements(self):
    self._rx(self._torque_driver_msg(100))
    self._rx(self._torque_driver_msg(-50))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(100, self.safety.get_torque_driver_max())
    # decay to 0
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())

  def test_curvature_measurements(self):
    self._rx(self._curvature_meas_msg(1500))
    self._rx(self._curvature_meas_msg(-500))
    self.assertEqual(-500, self.safety.get_curvature_meas_min())
    self.assertEqual(1500, self.safety.get_curvature_meas_max())

  def test_brake_signal(self):
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_brake_pressed_prev())


class TestVolkswagenMebStockSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [
    [MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0],
    [MSG_EA_01, 0], [MSG_EA_02, 0], [MSG_KLR_01, 0], [MSG_KLR_01, 2]
  ]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03], 2: [MSG_HCA_03, MSG_LDW_02]}

  def setUp(self):
    self.packer = CANPackerPanda("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, 0)
    self.safety.init_tests()

  def test_cancel_only_when_controls_off(self):
    self.safety.set_controls_allowed(0)
    # cancel allowed
    self.assertTrue(self._tx(self._gra_acc_01_msg(cancel=1)))
    # set/resume blocked
    self.assertFalse(self._tx(self._gra_acc_01_msg(resume=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(_set=1)))


class TestVolkswagenMebLongSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [
    [MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_ACC_18, 0],
    [MSG_MEB_ACC_01, 0], [MSG_TA_01, 0], [MSG_EA_01, 0], [MSG_EA_02, 0],
    [MSG_KLR_01, 0], [MSG_KLR_01, 2]
  ]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03], 2: [MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_MEB_ACC_01]}

  def setUp(self):
    self.packer = CANPackerPanda("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  def test_set_and_resume_buttons(self):
    for button in ["set", "resume"]:
      self.safety.set_controls_allowed(0)
      self._rx(self._tsk_status_msg(False, main_switch=True))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume")))
      self.assertFalse(self.safety.get_controls_allowed())
      # falling edge
      self._rx(self._gra_acc_01_msg())
      self.assertTrue(self.safety.get_controls_allowed())

  def test_cancel_button(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._gra_acc_01_msg(cancel=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_switch(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_accel_safety_check(self):
    for controls_allowed in [True, False]:
      for accel in np.concatenate((np.arange(MIN_ACCEL-1, MAX_ACCEL+1, 0.1), [0, INACTIVE_ACCEL, ACCEL_OVERRIDE])):
        accel = round(accel, 2)
        send = ((controls_allowed and MIN_ACCEL <= accel <= MAX_ACCEL) or accel == INACTIVE_ACCEL)
        self.safety.set_controls_allowed(controls_allowed)
        self.assertEqual(send, self._tx(self._accel_msg(accel)), (controls_allowed, accel))


if __name__ == "__main__":
  unittest.main()
