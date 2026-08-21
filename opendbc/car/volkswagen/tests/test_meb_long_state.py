from types import SimpleNamespace
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.volkswagen import mebcan
from opendbc.car.volkswagen.values import CAR, DBC, CarControllerParams


LongCtrlState = structs.CarControl.Actuators.LongControlState


class TestMebLongStateMachine(unittest.TestCase):
  PLATFORMS = (
    CAR.VOLKSWAGEN_ID3_MK1,
    CAR.VOLKSWAGEN_ID3_MK2,
    CAR.VOLKSWAGEN_GOLF_MK8,
    CAR.AUDI_A3_MK4,
  )

  @staticmethod
  def make_state_machine(platform):
    CP = SimpleNamespace(carFingerprint=platform, flags=platform.config.flags)
    return mebcan.SunnypilotMebLongStateMachine(CP, CarControllerParams(CP))

  def test_driver_torque_max_matches_three_nm(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform):
        CP = SimpleNamespace(carFingerprint=platform, flags=platform.config.flags)
        self.assertEqual(CarControllerParams(CP).STEER_DRIVER_MAX, 300)

  @staticmethod
  def make_car_state(v_ego=10.0, available=True, acc_faulted=False, stock_aeb=False,
                     gas_pressed=False, brake_pressed=False, esp_hold=False):
    out = SimpleNamespace(
      vEgo=v_ego,
      cruiseState=SimpleNamespace(available=available),
      accFaulted=acc_faulted,
      stockAeb=stock_aeb,
      gasPressed=gas_pressed,
      brakePressed=brake_pressed,
    )
    return SimpleNamespace(out=out, esp_hold_confirmation=esp_hold)

  @staticmethod
  def make_car_control(enabled=True, long_active=True, override=False, long_state=LongCtrlState.pid):
    return SimpleNamespace(
      enabled=enabled,
      longActive=long_active,
      cruiseControl=SimpleNamespace(override=override),
      actuators=SimpleNamespace(longControlState=long_state),
    )

  def test_disengage_clears_actionable_bits(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform):
        state = self.make_state_machine(platform)
        CS = self.make_car_state(v_ego=0.3)
        CC = self.make_car_control(long_state=LongCtrlState.stopping)

        accel, status, hold_type, braking_to_stop, leaving_standstill, held = state.update(CS, CC, -1.0)
        self.assertEqual(status, mebcan.ACC_CTRL_ACTIVE)
        self.assertEqual(hold_type, mebcan.ACC_HMS_HOLD)
        self.assertTrue(braking_to_stop)
        self.assertFalse(leaving_standstill)
        self.assertFalse(held)

        # controlsd can expose the previous stopping state for one transition
        # frame. No actionable bit may survive once longActive is false.
        CC.enabled = False
        CC.longActive = False
        accel, status, hold_type, braking_to_stop, leaving_standstill, held = state.update(CS, CC, -1.0)
        self.assertEqual(accel, state.CCP.ACCEL_INACTIVE)
        self.assertEqual(status, mebcan.ACC_CTRL_ENABLED)
        self.assertEqual(hold_type, mebcan.ACC_HMS_RAMP_RELEASE)
        self.assertFalse(braking_to_stop)
        self.assertFalse(leaving_standstill)
        self.assertFalse(held)
        self.assertFalse(state.acc_enabled)

  def test_hold_release_ramps_until_five_kph(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform):
        state = self.make_state_machine(platform)
        CS = self.make_car_state(v_ego=0.0)
        CC = self.make_car_control(long_state=LongCtrlState.stopping)
        state.update(CS, CC, -1.0)

        CS.esp_hold_confirmation = True
        CC.actuators.longControlState = LongCtrlState.pid
        accel, _, hold_type, _, leaving_standstill, _ = state.update(CS, CC, 0.2)
        self.assertEqual(accel, state.CCP.STARTING_ACCEL)
        self.assertEqual(hold_type, mebcan.ACC_HMS_RELEASE)
        self.assertTrue(leaving_standstill)

        CS.esp_hold_confirmation = False
        CS.out.vEgo = 1.0  # 3.6 km/h, above the fork launch latch but below ramp release speed
        _, _, hold_type, braking_to_stop, leaving_standstill, _ = state.update(CS, CC, 0.2)
        self.assertEqual(hold_type, mebcan.ACC_HMS_RAMP_RELEASE)
        self.assertFalse(braking_to_stop)
        self.assertFalse(leaving_standstill)

        CS.out.vEgo = 1.5  # 5.4 km/h
        _, _, hold_type, _, _, _ = state.update(CS, CC, 0.2)
        self.assertEqual(hold_type, mebcan.ACC_HMS_NO_REQUEST)

  def test_enabled_without_long_active_stays_inactive(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform):
        state = self.make_state_machine(platform)
        CS = self.make_car_state()
        CC = self.make_car_control(enabled=True, long_active=False, override=True)
        accel, status, hold_type, braking_to_stop, leaving_standstill, _ = state.update(CS, CC, 0.5)

        self.assertEqual(accel, state.CCP.ACCEL_INACTIVE)
        self.assertEqual(status, mebcan.ACC_CTRL_ENABLED)
        self.assertEqual(hold_type, mebcan.ACC_HMS_NO_REQUEST)
        self.assertFalse(braking_to_stop)
        self.assertFalse(leaving_standstill)
        self.assertFalse(state.acc_enabled)

  def test_gas_override_and_stock_aeb(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform):
        state = self.make_state_machine(platform)
        CS = self.make_car_state(gas_pressed=True)
        CC = self.make_car_control(enabled=True, long_active=False, override=True)
        accel, status, _, braking_to_stop, leaving_standstill, _ = state.update(CS, CC, 0.5)
        self.assertEqual(accel, state.CCP.ACCEL_OVERRIDE)
        self.assertEqual(status, mebcan.ACC_CTRL_OVERRIDE)
        self.assertFalse(braking_to_stop)
        self.assertFalse(leaving_standstill)
        self.assertTrue(state.acc_enabled)
        self.assertEqual(state.comfort_accel, state.CCP.ACCEL_OVERRIDE)

        state = self.make_state_machine(platform)
        CS = self.make_car_state(stock_aeb=True)
        CC = self.make_car_control(enabled=True, long_active=True)
        accel, status, _, braking_to_stop, leaving_standstill, _ = state.update(CS, CC, 0.5)
        self.assertEqual(accel, state.CCP.ACCEL_INACTIVE)
        self.assertEqual(status, mebcan.ACC_CTRL_ENABLED)
        self.assertFalse(braking_to_stop)
        self.assertFalse(leaving_standstill)
        self.assertFalse(state.acc_enabled)
        self.assertEqual(state.comfort_accel, 0.0)

  def test_aeb_inactive_field_split_preserves_wire_value(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform):
        CP = SimpleNamespace(carFingerprint=platform, flags=platform.config.flags)
        _, data, _ = mebcan.create_aeb_control(CANPacker(DBC[platform][Bus.pt]), 0, CP)

        # AWV_Unavailable is bit 17 and remains clear in the replacement.
        self.assertEqual((int.from_bytes(data, "little") >> 17) & 1, 0)
        # The adjacent explicit SET_ME_63 field preserves the old combined
        # seven-bit inactive wire value of 126.
        self.assertEqual((int.from_bytes(data, "little") >> 17) & 0x7F, 126)

  def test_disengage_message_matches_safety_contract(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform):
        CP = SimpleNamespace(carFingerprint=platform, flags=platform.config.flags)
        state = mebcan.SunnypilotMebLongStateMachine(CP, CarControllerParams(CP))
        CS = self.make_car_state(v_ego=0.3)
        CC = self.make_car_control(long_state=LongCtrlState.stopping)
        state.update(CS, CC, -1.0)

        CC.enabled = False
        CC.longActive = False
        accel, status, hold_type, braking_to_stop, leaving_standstill, held = state.update(CS, CC, -1.0)
        messages = mebcan.create_acc_accel_control(
          CANPacker(DBC[platform][Bus.pt]), 0, CP, 2, state.acc_enabled,
          4.0, 4.0, 0.0, 0.0, accel, status, hold_type,
          braking_to_stop, leaving_standstill, held, CS.out.vEgo * 3.6, False,
        )

        address, data, bus = messages[0]
        self.assertEqual((address, len(data), bus), (0x14D, 32, 0))
        self.assertEqual((data[7] >> 4) & 0x7, mebcan.ACC_CTRL_ENABLED)
        self.assertEqual(data[7] & 0x3, 0)  # ACC_Anfahren / ACC_Anhalten
        self.assertEqual((data[9] >> 5) & 0x7, mebcan.ACC_HMS_RAMP_RELEASE)


if __name__ == "__main__":
  unittest.main()
