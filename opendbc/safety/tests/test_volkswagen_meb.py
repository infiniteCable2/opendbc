#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.car.lateral import ISO_LATERAL_ACCEL, ISO_LATERAL_JERK

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


class TestVolkswagenMebSafetyBase(common.PandaCarSafetyTest, common.CurvatureSteeringSafetyTest):
  # === limits ===
  MAX_CURVATURE = 29105
  CURVATURE_TO_CAN = 149253.7313
  INACTIVE_CURVATURE_IS_ZERO = True
  MAX_POWER = 125  # 50% bei (0.4,0) Skalierung -> 50/0.4 = 125
  SEND_RATE = 0.02

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}

  # Wheel speeds
  def _speed_msg(self, speed_mps: float):
    spd_kph = speed_mps * 3.6
    values = { "HL_Radgeschw": spd_kph, "HR_Radgeschw": spd_kph, "VL_Radgeschw": spd_kph, "VR_Radgeschw": spd_kph}
    return self.packer.make_can_msg_panda("ESC_51", 0, values)

  # Brake pedal switch
  def _motor_14_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_panda("Motor_14", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_14_msg(brake)

  # Driver throttle input
  def _user_gas_msg(self, gas):
    values = {"Accelerator_Pressure": gas}
    return self.packer.make_can_msg_panda("Motor_54", 0, values)

  def _vehicle_moving_msg(self, speed_mps: float):
    return self._speed_msg(speed_mps)

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
    
  # ACC engagement status
  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    values = {"TSK_Status": tsk_status}
    return self.packer.make_can_msg_panda("Motor_51", 0, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_panda("LH_EPS_03", 0, values)

  # Cruise control buttons
  def _button_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_panda("GRA_ACC_01", bus, values)

  def _accel_msg(self, accel):
    values = {"ACC_Sollbeschleunigung_02": accel}
    return self.packer.make_can_msg_panda("ACC_18", 0, values)

  def setUp(self):
    self.packer = CANPackerPanda("vw_meb")

  def test_curvature_measurements(self):
    self._rx(self._curvature_meas_msg(1500))
    self._rx(self._curvature_meas_msg(-500))
    self._rx(self._curvature_meas_msg(0))
    self._rx(self._curvature_meas_msg(0))
    self._rx(self._curvature_meas_msg(0))
    self._rx(self._curvature_meas_msg(0))

    self.assertEqual(-500, self.safety.get_curvature_meas_min())
    self.assertEqual(1500, self.safety.get_curvature_meas_max())

    self._rx(self._curvature_meas_msg(0))
    self.assertEqual(0, self.safety.get_curvature_meas_max())
    self.assertEqual(-500, self.safety.get_curvature_meas_min())
    
    self._rx(self._curvature_meas_msg(0))
    self.assertEqual(0, self.safety.get_curvature_meas_max())
    self.assertEqual(0, self.safety.get_curvature_meas_min())

  def test_brake_signal(self):
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_brake_pressed_prev())


class TestVolkswagenMebStockSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0],
             [MSG_EA_01, 0], [MSG_EA_02, 0], [MSG_KLR_01, 0], [MSG_KLR_01, 2]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03], 2: [MSG_HCA_03, MSG_LDW_02]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}

  def setUp(self):
    self.packer = CANPackerPanda("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=1)))
    self.assertFalse(self._tx(self._button_msg(resume=1)))
    self.assertFalse(self._tx(self._button_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(resume=1)))


class TestVolkswagenMebCurvatureSafety(TestVolkswagenMebSafetyBase, common.CurvatureSteeringSafetyTest):
  TX_MSGS = [[MSG_HCA_03, 0]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03], 2: [MSG_HCA_03, MSG_LDW_02]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}

  def setUp(self):
    self.packer = CANPackerPanda("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, 0)
    self.safety.init_tests()
    

class TestVolkswagenMebLongSafety(TestVolkswagenMebSafetyBase):
  ALLOW_OVERRIDE = True
  
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_ACC_18, 0],
             [MSG_MEB_ACC_01, 0], [MSG_TA_01, 0], [MSG_EA_01, 0], [MSG_EA_02, 0],
             [MSG_KLR_01, 0], [MSG_KLR_01, 2]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03],
                           2: [MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_MEB_ACC_01]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_MEB_ACC_01)}
  
  def setUp(self):
    self.packer = CANPackerPanda("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ["set", "resume"]:
      # ACC main switch must be on, engage on falling edge
      self.safety.set_controls_allowed(0)
      self._rx(self._tsk_status_msg(False, main_switch=False))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      self._rx(self._tsk_status_msg(False, main_switch=True))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      self._rx(self._button_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    # Disable as soon as main switch turns off
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_accel_safety_check(self):
    for controls_allowed in [True, False]:
      # enforce we don't skip over 0 or inactive accel
      for accel in np.concatenate((np.arange(MIN_ACCEL - 2, MAX_ACCEL + 2, 0.03), [0, INACTIVE_ACCEL])):
        accel = round(accel, 2)  # floats might not hit exact boundary conditions without rounding
        is_inactive_accel = accel == self.INACTIVE_ACCEL
        send = (controls_allowed and MIN_ACCEL <= accel <= MAX_ACCEL) or is_inactive_accel
        self.safety.set_controls_allowed(controls_allowed)
        # accel request used by ECU
        self.assertEqual(send, self._tx(self._acc_18_msg(accel)), (controls_allowed, accel))

  def test_accel_override_with_gas(self):
    if not self.ALLOW_OVERRIDE:
      pass
    self.safety.set_controls_allowed(True)
    self.safety.set_gas_pressed_prev(True)
    # override accel has to be spcific value
    self.assertTrue(self._tx(self._accel_msg(ACCEL_OVERRIDE)))
    self.assertFalse(self._tx(self._accel_msg(MAX_ACCEL)))


if __name__ == "__main__":
  unittest.main()
