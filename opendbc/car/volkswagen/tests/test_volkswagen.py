import random
import re
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, DBC, CarControllerParams as CCP, FW_QUERY_CONFIG, VolkswagenFlags, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        assert hca_mitigation.update(actuator_value, actuator_value) == expected_torque, f"{frame=}"


class TestVolkswagenStandstillSignals(unittest.TestCase):
  @staticmethod
  def make_interface(platform):
    fingerprint = {bus: {} for bus in range(4)}
    cp = CarInterface.get_params(platform, fingerprint, [], False, False, False)
    cp_sp = CarInterface.get_params_sp(cp, platform, fingerprint, [], False, False, False)
    cp_ic = CarInterface.get_params_ic(cp, platform, fingerprint, [], False, False, False)
    return CarInterface(cp, cp_sp, cp_ic), CANPacker(DBC[platform][Bus.pt])

  def test_meb_motion_state_and_gen2_hms(self):
    for platform in (CAR.CUPRA_BORN_MK1, CAR.VOLKSWAGEN_ID3_MK2):
      with self.subTest(platform=platform):
        ci, packer = self.make_interface(platform)
        for motion_state in range(4):
          msg = packer.make_can_msg("ESC_50", 0, {"Motion_State": motion_state})
          ci.update([(motion_state + 1, [msg])])
          self.assertEqual(ci.CS.esp_standstill, motion_state == 3)
          self.assertEqual(ci.CS.esp_hold_confirmation, motion_state == 3 if platform == CAR.CUPRA_BORN_MK1 else False)

        if platform == CAR.VOLKSWAGEN_ID3_MK2:
          for hms_status in (0, 1, 2):
            msg = packer.make_can_msg("VMM_02", 0, {"HMS_Status": hms_status})
            ci.update([(hms_status + 5, [msg])])
            self.assertEqual(ci.CS.esp_hold_confirmation, hms_status == 1)

  def test_mqb_evo_standstill(self):
    ci, packer = self.make_interface(CAR.VOLKSWAGEN_GOLF_MK8)
    for standstill in (0, 1):
      msg = packer.make_can_msg("ESC_50", 0, {"Standstill": standstill})
      ci.update([(standstill + 1, [msg])])
      self.assertEqual(ci.CS.esp_standstill, bool(standstill))


class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_flag_values_are_unique(self):
    values = [flag.value for flag in VolkswagenFlags.__members__.values()]
    self.assertEqual(len(values), len(set(values)))

  def test_bsm_does_not_change_platform(self):
    fingerprint = {bus: {} for bus in range(4)}
    fingerprint[0][0x24C] = 8  # MEB_Side_Assist_01
    meb_cp = CarInterface.get_params(CAR.CUPRA_BORN_MK1, fingerprint, [], False, False, False)
    self.assertTrue(meb_cp.flags & VolkswagenFlags.HAS_BSM)
    self.assertFalse(meb_cp.flags & VolkswagenFlags.MQB_EVO)

    fingerprint[0].clear()
    mqb_evo_cp = CarInterface.get_params(CAR.VOLKSWAGEN_GOLF_MK8, fingerprint, [], False, False, False)
    self.assertTrue(mqb_evo_cp.flags & VolkswagenFlags.MQB_EVO)
    self.assertFalse(mqb_evo_cp.flags & VolkswagenFlags.HAS_BSM)

  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    platforms = list(CAR)
    for i, platform in enumerate(platforms):
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # Platforms may share chassis codes when their model years disambiguate the VIN.
        for comp in platforms[i + 1:]:
          if not (platform.config.wmis & comp.config.wmis and
                  platform.config.chassis_codes & comp.config.chassis_codes):
            continue

          model_years_overlap = (not platform.config.model_years or not comp.config.model_years or
                                 bool(platform.config.model_years & comp.config.model_years))
          assert not model_years_overlap, f"Ambiguous VIN attributes: {platform} and {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      for wmi in WMI:
        for chassis_code in platform.config.chassis_codes | {"00"}:
          for model_year in platform.config.model_years | {"0"}:
            with self.subTest(platform=platform.name, wmi=wmi, chassis_code=chassis_code, model_year=model_year):
              vin = ["0"] * 17
              vin[0:3] = wmi
              vin[6:8] = chassis_code
              vin[9] = model_year
              vin = "".join(vin)

              # Check a few FW cases - expected, unexpected
              for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
                should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                                (not platform.config.model_years or model_year in platform.config.model_years) and
                                radar_fw in all_radar_fw)

                live_fws = {(0x757, None): [radar_fw]}
                matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

                expected_matches = {platform} if should_match else set()
                assert expected_matches == matches, "Bad match"
