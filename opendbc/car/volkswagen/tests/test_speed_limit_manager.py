import math
import unittest
from types import SimpleNamespace

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.speed_limit_manager import (
  NOT_SET, PSD_TYPE_CURV_SPEED, PSD_TYPE_SPEED_LIMIT, SpeedLimitManager,
)
from opendbc.car.volkswagen.values import VolkswagenFlags


def segment(segment_id, parent_id=0, length=100, likely=True, street_category=3, ramp=0,
            curvature_begin=255, curvature_end=255, identity_id=None):
  return {
    "PSD_Segment_ID": segment_id,
    "PSD_Vorgaenger_Segment_ID": parent_id,
    "PSD_Segmentlaenge": length,
    "PSD_Strassenkategorie": street_category,
    "PSD_Endkruemmung": curvature_end,
    "PSD_Endkruemmung_Vorz": 0,
    "PSD_Idenditaets_ID": segment_id if identity_id is None else identity_id,
    "PSD_ADAS_Qualitaet": 1,
    "PSD_wahrscheinlichster_Pfad": int(likely),
    "PSD_Geradester_Pfad": int(likely),
    "PSD_Bebauung": 0,
    "PSD_Segment_Komplett": 1,
    "PSD_Rampe": ramp,
    "PSD_Anfangskruemmung": curvature_begin,
    "PSD_Anfangskruemmung_Vorz": 0,
    "PSD_Abzweigerichtung": 0,
    "PSD_Abzweigewinkel": 0,
  }


def position(segment_id, remaining, unique=True):
  return {
    "PSD_Pos_Segment_ID": segment_id,
    "PSD_Pos_Segmentlaenge": remaining,
    "PSD_Pos_Standort_Eindeutig": int(unique),
  }


def speed_attribute(segment_id, raw_speed, offset=0):
  return {
    "PSD_06_Mux": 2,
    "PSD_Ges_Segment_ID": segment_id,
    "PSD_Ges_Offset": offset,
    "PSD_Ges_Geschwindigkeit": raw_speed,
    "PSD_Ges_Typ": 1,
    "PSD_Ges_Geschwindigkeit_Witter": 0,
    "PSD_Ges_Geschwindigkeit_Tag_Anf": 0,
    "PSD_Ges_Geschwindigkeit_Tag_Ende": 0,
    "PSD_Ges_Geschwindigkeit_Std_Anf": 25,
    "PSD_Ges_Geschwindigkeit_Std_Ende": 25,
    "PSD_Ges_Gesetzlich_Kategorie": 0,
  }


def legal_speed_attribute(street_type, raw_speed):
  attribute = speed_attribute(2, raw_speed)
  attribute["PSD_Ges_Typ"] = 2
  attribute["PSD_Ges_Gesetzlich_Kategorie"] = street_type
  return attribute


def vze(speed, sign_type=0):
  return {
    "VZE_Verkehrszeichen_1": speed,
    "VZE_Verkehrszeichen_1_Typ": sign_type,
    "VZE_Anzeigemodus": 0,
  }


class TestSpeedLimitManager(unittest.TestCase):
  def setUp(self):
    cp = SimpleNamespace(flags=VolkswagenFlags.MEB)
    self.manager = SpeedLimitManager(cp, predicative=True, predicative_speed_limit=True, predicative_curve=True)

  def update(self, speed_kph, psd_04=None, psd_05=None, psd_06=None, traffic_sign=None):
    self.manager.update(speed_kph * CV.KPH_TO_MS, psd_04 or {}, psd_05 or {}, psd_06 or {}, traffic_sign or {}, False, {})

  def add_current_limit(self, segment_id=2, limit_raw=16, remaining=100, **segment_kwargs):
    self.update(100, segment(segment_id, **segment_kwargs), position(segment_id, remaining))
    self.update(100, psd_05=position(segment_id, remaining), psd_06=speed_attribute(segment_id, limit_raw))

  def test_adasis_piecewise_curvature_decode(self):
    expected = {
      255: 0.0,
      191: 0.00064,
      190: 0.00066,
      127: 0.00192,
      126: 0.00196,
      63: 0.00448,
      62: 0.00456,
      0: 0.00952,
    }
    for raw, curvature in expected.items():
      with self.subTest(raw=raw):
        self.assertAlmostEqual(self.manager._get_segment_curvature_psd(raw, 0), curvature)
        self.assertAlmostEqual(self.manager._get_segment_curvature_psd(raw, 1), -curvature)

  def test_persistent_braking_event_is_passed_without_timeout(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2), position(2, 30), speed_attribute(3, 11))

    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 100)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)
    self.assertEqual(self.manager.get_speed_limit_predicative_type(), PSD_TYPE_SPEED_LIMIT)

    self.update(80, psd_05=position(3, 100))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)
    self.assertEqual(self.manager.get_speed_limit_predicative(), NOT_SET)

    for _ in range(20):
      self.update(50, psd_05=position(3, 80))
      self.manager.get_speed_limit()
      self.assertEqual(self.manager.get_speed_limit_predicative(), NOT_SET)

  def test_vze_mismatch_cannot_resurrect_old_map_limit(self):
    self.add_current_limit()
    self.update(100, psd_05=position(2, 50), traffic_sign=vze(50))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)

  def test_vze_mismatch_also_suppresses_legal_fallback(self):
    self.update(100, segment(2), position(2, 100), legal_speed_attribute(2, 16))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 100)

    self.update(100, psd_05=position(2, 80), traffic_sign=vze(50))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)
    self.update(50, psd_05=position(2, 70), traffic_sign=vze(0))
    self.assertEqual(self.manager.get_speed_limit(), NOT_SET)

    self.update(50, psd_05=position(2, 40), traffic_sign=vze(0))
    self.assertEqual(self.manager.get_speed_limit(), NOT_SET)

    self.update(50, segment(3, parent_id=2), position(2, 20), speed_attribute(3, 11))
    self.update(50, psd_05=position(3, 100), traffic_sign=vze(0))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)

  def test_recycled_id_creates_new_incarnation_without_speed_inheritance(self):
    self.add_current_limit()
    old_key = self.manager._current_key
    self.update(100, segment(3, parent_id=2), position(2, 80))
    self.update(100, segment(2, parent_id=3, length=60), position(2, 70))

    new_key = self.manager._active_by_id[2]
    self.assertNotEqual(old_key, new_key)
    self.assertEqual(self.manager.predicative_segments[old_key].speed, 100)
    self.assertEqual(self.manager.predicative_segments[new_key].speed, NOT_SET)

    self.update(100, psd_05=position(3, 50))
    self.assertNotIn(old_key, self.manager.predicative_segments)
    self.assertIn(new_key, self.manager.predicative_segments)

  def test_all_branches_are_retained_but_only_unique_likely_branch_is_selected(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, likely=True), position(2, 100))
    likely_key = self.manager._active_by_id[3]
    self.update(100, segment(4, parent_id=2, likely=False), position(2, 100))
    alternative_key = self.manager._active_by_id[4]
    self.manager._ensure_route_cache()

    self.assertIn(likely_key, self.manager.predicative_segments[self.manager._current_key].children)
    self.assertIn(alternative_key, self.manager.predicative_segments[self.manager._current_key].children)
    self.assertEqual(self.manager._route, (self.manager._current_key, likely_key))

  def test_curve_event_lives_for_exact_segment_not_timeout(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, curvature_begin=75, curvature_end=75), position(2, 30))
    curve_target = self.manager._calculate_curve_speed(self.manager._get_segment_curvature_psd(75, 0))
    self.assertLess(curve_target, 100)

    self.manager.get_speed_limit()
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, curve_target)
    self.assertEqual(self.manager.get_speed_limit_predicative_type(), PSD_TYPE_CURV_SPEED)

    self.update(curve_target, psd_05=position(3, 80))
    self.manager.get_speed_limit()
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, curve_target)

    self.update(curve_target, segment(4, parent_id=3), position(3, 10))
    self.update(curve_target, psd_05=position(4, 100))
    self.manager.get_speed_limit()
    self.assertEqual(self.manager.get_speed_limit_predicative(), NOT_SET)

  def test_speed_offset_uses_psd05_remaining_distance(self):
    self.add_current_limit(remaining=100)
    self.update(100, psd_05=position(2, 80), psd_06=speed_attribute(2, 11, offset=50))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 100)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    self.update(80, psd_05=position(2, 50))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)
    self.assertEqual(self.manager.get_speed_limit_predicative(), NOT_SET)

  def test_late_speed_unit_update_redecodes_stored_attributes(self):
    self.add_current_limit(limit_raw=11)
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)

    unit_message = {
      "PSD_06_Mux": 0,
      "PSD_Sys_Segment_ID": 2,
      "PSD_Sys_Geschwindigkeit_Einheit": 1,
    }
    self.update(50, psd_05=position(2, 80), psd_06=unit_message)
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 40 * CV.MPH_TO_KPH)

  def test_route_extension_does_not_invalidate_committed_event(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)
    committed = self.manager._committed_event_identity

    self.update(100, segment(4, parent_id=3), position(2, 28))
    self.assertEqual(self.manager._committed_event_identity, committed)
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

  def test_prediction_fails_closed_for_ambiguous_position(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    self.update(100, psd_05=position(2, 30, unique=False))
    self.manager.get_speed_limit()
    self.assertEqual(self.manager.get_speed_limit_predicative(), NOT_SET)

  def test_feature_switch_rebuilds_events_and_clears_commit(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    self.manager.enable_predicative_speed_limit(False, False, False)
    self.update(100, psd_05=position(2, 28))
    self.manager.get_speed_limit()
    self.assertEqual(self.manager.get_speed_limit_predicative(), NOT_SET)
    self.assertIsNone(self.manager._committed_event_identity)

    self.manager.enable_predicative_speed_limit(True, True, False)
    self.update(100, psd_05=position(2, 26))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

  def test_curve_speed_matches_lateral_acceleration_envelope(self):
    curvature = self.manager._get_segment_curvature_psd(75, 0)
    speed_kph = self.manager._calculate_curve_speed(curvature)
    self.assertLessEqual((speed_kph * CV.KPH_TO_MS) ** 2 * curvature, 3.1)
    self.assertTrue(math.isfinite(speed_kph))


if __name__ == "__main__":
  unittest.main()
