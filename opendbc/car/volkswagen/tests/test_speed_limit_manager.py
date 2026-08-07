import math
import unittest
from types import SimpleNamespace
from unittest import mock

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.speed_limit_manager import (
  CURVE_PROFILE_STEP_M, DECELERATION_PREDICATIVE, NOT_SET, PSD_05_FIELDS,
  PSD_TYPE_CURV_SPEED, PSD_TYPE_SPEED_LIMIT, SPEED_SEGMENT_PROTECTION_TIME_S, SpeedLimitManager,
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


def position(segment_id, remaining, unique=True, longitudinal_error=None):
  if longitudinal_error is None:
    longitudinal_error = 1 if unique else 6
  return {
    "PSD_Pos_Segment_ID": segment_id,
    "PSD_Pos_Segmentlaenge": remaining,
    "PSD_Pos_Inhibitzeit": 200,
    "PSD_Pos_Standort_Eindeutig": int(unique),
    "PSD_Pos_Fehler_Laengsrichtung": longitudinal_error,
  }


def empty_position_frame():
  return {name: 0 for name in PSD_05_FIELDS}


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

  def test_offroute_branch_mutation_keeps_selected_event_profile(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, likely=True, curvature_begin=75, curvature_end=75), position(2, 80))
    self.manager._ensure_route_cache()
    events = self.manager._events
    self.assertTrue(events)

    self.update(100, segment(4, parent_id=2, likely=False), position(2, 79))

    self.assertIs(self.manager._events, events)
    self.assertEqual(self.manager._route[-1], self.manager._active_by_id[3])

  def test_selected_route_event_change_rebuilds_profile(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2), position(2, 80), speed_attribute(3, 11))
    events = self.manager._events

    self.update(100, psd_05=position(2, 79), psd_06=speed_attribute(3, 10))

    self.assertIsNot(self.manager._events, events)
    selected_speed = next(event.speed for event in self.manager._events if event.segment_key == self.manager._active_by_id[3])
    self.assertEqual(selected_speed, 45)

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

    # While still in the curve segment, the cap persists.
    self.update(curve_target, psd_05=position(3, 10))
    self.manager.get_speed_limit()
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, curve_target)

    # Leaving the curve segment onto a straight segment releases the cap
    # immediately instead of ramping it out over time.
    self.update(curve_target, segment(4, parent_id=3), position(3, 0))
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

  def test_prediction_freezes_committed_cap_for_ambiguous_position(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    target = self.manager.get_speed_limit_predicative()
    committed = self.manager._committed_event_identity
    self.assertGreater(target, 0)

    self.update(100, psd_05=position(2, 30, unique=False))
    self.manager.get_speed_limit()
    self.assertEqual(self.manager.get_speed_limit_predicative(), target)
    self.assertEqual(self.manager._committed_event_identity, committed)

  def test_full_zero_psd05_record_is_a_semantic_noop(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    target = self.manager.get_speed_limit_predicative()
    current_key = self.manager._current_key
    committed = self.manager._committed_event_identity

    self.update(100, psd_05=empty_position_frame())
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._last_psd_05_frame_empty)
    self.assertTrue(self.manager._current_valid)
    self.assertEqual(self.manager._current_key, current_key)
    self.assertEqual(self.manager._committed_event_identity, committed)
    self.assertEqual(self.manager.get_speed_limit_predicative(), target)

  def test_explicit_offroad_position_invalidates_prediction(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    self.update(100, psd_05=position(0, 0, longitudinal_error=7))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._current_valid)
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

  def test_curve_corridor_is_piecewise_and_keeps_five_kph_steps(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=100, curvature_begin=255, curvature_end=75), position(2, 100))
    self.manager._ensure_route_cache()
    curve_events = [event for event in self.manager._events if event.segment_key == self.manager._active_by_id[3] and
                    event.event_type == PSD_TYPE_CURV_SPEED]

    self.assertLessEqual(max(event.end_offset - event.offset for event in curve_events), CURVE_PROFILE_STEP_M)
    self.assertGreater(len({event.speed for event in curve_events}), 1)
    self.assertTrue(all(event.speed % 5 == 0 for event in curve_events))
    self.assertTrue(all(a.speed >= b.speed for a, b in zip(curve_events, curve_events[1:], strict=False)))

  def test_event_cursor_only_scans_the_braking_horizon(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=1000, curvature_begin=75, curvature_end=75), position(2, 30))
    self.manager._ensure_route_cache()
    event_count = len(self.manager._events)
    self.assertGreater(event_count, 150)

    with mock.patch.object(self.manager, "_indexed_event_distance", wraps=self.manager._indexed_event_distance) as distance:
      self.update(50, psd_05=position(2, 29))

    self.assertLess(distance.call_count, 30)
    self.assertLess(distance.call_count, event_count)

  def test_event_cursor_resets_after_position_regression(self):
    self.update(100, segment(2, length=100, curvature_begin=75, curvature_end=75), position(2, 80))
    self.update(100, psd_05=position(2, 80), psd_06=speed_attribute(2, 16))
    cursor_before_regression = self.manager._event_cursor
    self.assertGreater(cursor_before_regression, 0)

    self.update(100, psd_05=position(2, 95))
    self.assertLess(self.manager._event_cursor, cursor_before_regression)
    event = self.manager._events[self.manager._event_cursor]
    self.assertGreaterEqual(self.manager._event_end_position(self.manager._event_cursor), 5)
    self.assertEqual(event.event_type, PSD_TYPE_CURV_SPEED)

  def test_committed_event_survives_outside_current_braking_window(self):
    self.add_current_limit(remaining=100)
    self.update(130, segment(3, parent_id=2, length=800), position(2, 100), speed_attribute(3, 11, offset=590))
    self.manager.get_speed_limit()
    committed = self.manager._committed_event_identity
    self.assertIsNotNone(committed)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    with mock.patch.object(self.manager, "_indexed_event_distance", wraps=self.manager._indexed_event_distance) as distance:
      self.update(50, psd_05=position(2, 100))

    self.manager.get_speed_limit()
    self.assertEqual(self.manager._committed_event_identity, committed)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)
    committed_index = self.manager._committed_event_index
    self.assertIsNotNone(committed_index)
    self.assertGreater(self.manager._event_start_positions[committed_index], 100 ** 2 * CV.KPH_TO_MS ** 2 / (2 * DECELERATION_PREDICATIVE))
    self.assertLessEqual(distance.call_count, 2)

  def test_curve_cap_is_passed_through_directly_without_ramp(self):
    # A curve segment that tapers from strong curvature to straight: the cap
    # must follow the piecewise profile directly, without jerk/accel ramping.
    self.update(100, segment(2, length=200, street_category=3), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 16))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 100)

    self.update(100, segment(3, parent_id=2, length=400, street_category=3,
                             curvature_begin=50, curvature_end=255), position(2, 30))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    # Drive through the curve; the cap should follow the profile directly.
    for remaining in [350, 300, 250, 200, 150, 100, 50, 1]:
      self.update(80, psd_05=position(3, remaining))
      self.manager.get_speed_limit()
      target = self.manager.get_speed_limit_predicative()
      if target != NOT_SET:
        # No ramp: each step must be a clean 5-kph quantum from the profile.
        self.assertAlmostEqual(target * CV.MS_TO_KPH % 5, 0, delta=0.1)

  def test_false_large_vze_drop_is_rejected_stickily(self):
    self.add_current_limit(limit_raw=11)
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)

    self.update(50, psd_05=position(2, 80), traffic_sign=vze(10))
    self.assertTrue(self.manager.v_limit_vze_sanity_error)
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)

    self.manager.v_limit_output_last = NOT_SET
    self.update(50, psd_05=position(2, 70), traffic_sign=vze(10))
    self.assertTrue(self.manager.v_limit_vze_sanity_error)
    self.assertEqual(self.manager.v_limit_vze, NOT_SET)

    self.update(50, psd_05=position(2, 60), traffic_sign=vze(50))
    self.assertFalse(self.manager.v_limit_vze_sanity_error)
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 50)

  def test_speed_protection_keeps_psd_trusted_when_vze_lags(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=200), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    # Arrive at the target segment while VZE still reports the old (higher) limit.
    self.update(50, psd_05=position(3, 200), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertEqual(self.manager._speed_protection_speed, 50)
    # Predicative output is held at 50; normal output reflects VZE (100).
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

  def test_speed_protection_releases_after_distance_exceeded(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=200), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()

    self.update(50, psd_05=position(3, 200), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    # Drive beyond the speed-derived protection distance while VZE still lags.
    # The grace period ends, predicative output is no longer held.
    protection_distance = 50 * CV.KPH_TO_MS * SPEED_SEGMENT_PROTECTION_TIME_S
    remaining = 200 - protection_distance - 1
    self.update(50, psd_05=position(3, max(0, remaining)), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

  def test_speed_protection_releases_when_vze_changes(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=200), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()

    self.update(50, psd_05=position(3, 200), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    # VZE changes to confirm the PSD limit: VZE has changed (from 100 to 50),
    # so the protection ends. Predicative is no longer held (VZE matches PSD).
    self.update(50, psd_05=position(3, 180), traffic_sign=vze(50))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

  def test_speed_protection_persists_while_vze_lags_at_old_limit(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=200), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()

    self.update(50, psd_05=position(3, 200), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    # VZE keeps reporting the old limit (100) — protection stays.
    self.update(50, psd_05=position(3, 180), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    self.update(50, psd_05=position(3, 170), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    # VZE changes to a different limit: protection ends.
    self.update(50, psd_05=position(3, 160), traffic_sign=vze(80))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

  def test_speed_protection_releases_when_psd_limit_changes(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=200), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()

    self.update(50, psd_05=position(3, 200), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    # Next segment carries a different PSD limit: protection ends.
    self.update(50, segment(4, parent_id=3, length=200), position(3, 10),
                psd_06=speed_attribute(4, 16))
    self.update(50, psd_05=position(4, 200), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

  def test_speed_protection_does_not_break_predicative_for_following_lower_event(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=200), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    self.update(50, psd_05=position(3, 200), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    # A follow-on segment carries an even lower limit; the predicative path
    # must still pick it up despite the active speed protection.
    self.update(50, segment(4, parent_id=3, length=200), position(3, 30), speed_attribute(4, 8))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)
    self.assertEqual(self.manager.get_speed_limit_predicative_type(), PSD_TYPE_SPEED_LIMIT)

  def test_curve_cap_releases_immediately_when_curvature_ends_on_highway_entry(self):
    # Highway on-ramp with a curve, then a straight highway segment.
    self.update(100, segment(2, length=120, street_category=5), position(2, 120))
    self.update(100, psd_05=position(2, 120), psd_06=speed_attribute(2, 23))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 130)

    # On-ramp segment with curvature.
    self.update(100, segment(3, parent_id=2, length=80, street_category=5, ramp=1,
                             curvature_begin=75, curvature_end=75), position(2, 30))
    self.manager.get_speed_limit()
    curve_target = self.manager._calculate_curve_speed(self.manager._get_segment_curvature_psd(75, 0))
    self.assertLess(curve_target, 130)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, curve_target)
    self.assertEqual(self.manager.get_speed_limit_predicative_type(), PSD_TYPE_CURV_SPEED)

    # Enter the curve segment and drive through it.
    self.update(curve_target, psd_05=position(3, 80))
    self.manager.get_speed_limit()
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, curve_target)

    # Move onto the straight highway segment. The curve event is no longer in
    # the route, so the cap must release immediately instead of ramping out.
    self.update(curve_target, segment(4, parent_id=3, length=200, street_category=5),
                position(3, 0))
    self.update(curve_target, psd_05=position(4, 200))
    self.manager.get_speed_limit()
    self.assertEqual(self.manager.get_speed_limit_predicative(), NOT_SET)

  def test_vze_fluctuation_after_psd_confirmation_cannot_push_output_back_up(self):
    # PSD detects a 50 zone early; we brake to it. VZE lags at first, then
    # briefly confirms 50, then fluctuates back to the old limit before
    # settling. The output must stay at 50 throughout, never jumping back up.
    self.update(100, segment(2, length=200, street_category=5), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 23))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 130)

    # PSD sees the 50 limit early on a follow-on segment.
    self.update(100, segment(3, parent_id=2, length=200, street_category=3), position(2, 30),
                psd_06=speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    # Brake into the 50 segment; VZE still reports the old limit.
    self.update(50, psd_05=position(3, 200), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

    # VZE briefly confirms 50: VZE has changed (from 130 to 50), protection ends.
    self.update(50, psd_05=position(3, 180), traffic_sign=vze(50))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

    # VZE fluctuates back to the old limit: protection already ended, VZE takes priority.
    self.update(50, psd_05=position(3, 160), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

  def test_curve_protection_shields_cap_from_lagging_vze(self):
    # A curve is detected predictively; while braking towards it, VZE still
    # reports a higher limit. The curve cap must be protected from VZE.
    self.update(100, segment(2, length=200, street_category=5), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 23))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 130)

    self.update(100, segment(3, parent_id=2, length=200, street_category=3,
                             curvature_begin=50, curvature_end=50), position(2, 30))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)
    self.assertEqual(self.manager.get_speed_limit_predicative_type(), PSD_TYPE_CURV_SPEED)
    curve_target = self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH

    # Arrive at the curve segment; VZE still reports the old limit.
    self.update(curve_target, psd_05=position(3, 200), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    # Predicative output is held at the curve cap; normal output reflects VZE.
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, curve_target)

  def test_curve_protection_releases_when_vze_changes(self):
    self.update(100, segment(2, length=200, street_category=5), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 23))
    self.update(100, segment(3, parent_id=2, length=200, street_category=3,
                             curvature_begin=50, curvature_end=50), position(2, 30))
    self.manager.get_speed_limit()
    curve_target = self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH

    self.update(curve_target, psd_05=position(3, 200), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)

    # VZE changes to a different value: protection ends.
    self.update(curve_target, psd_05=position(3, 180), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

  def test_curve_events_generated_for_urban_segments(self):
    # Curves are now enabled for all street types, including urban.
    self.update(100, segment(2, length=200, street_category=1), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 16))
    self.update(100, segment(3, parent_id=2, length=200, street_category=1,
                             curvature_begin=50, curvature_end=50), position(2, 30))
    self.manager._ensure_route_cache()
    curve_events = [event for event in self.manager._events
                    if event.event_type == PSD_TYPE_CURV_SPEED]
    self.assertTrue(curve_events)

  def test_speed_protection_normal_limit_reflects_vze_while_predicative_holds(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=200), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    # Arrive at the target segment while VZE still reports the old (higher) limit.
    self.update(50, psd_05=position(3, 200), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    # Normal limit output reflects VZE (100), not the protected PSD limit (50).
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 100)
    # Predicative output is held at 50.
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)

  def test_curve_protection_not_ended_by_psd_speed_limit_change(self):
    self.update(100, segment(2, length=200, street_category=5), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 23))
    self.update(100, segment(3, parent_id=2, length=500, street_category=3,
                             curvature_begin=50, curvature_end=50), position(2, 30))
    self.manager.get_speed_limit()
    curve_target = self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH

    # Arrive at the curve segment; VZE still reports the old limit.
    self.update(curve_target, psd_05=position(3, 500), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertTrue(self.manager._speed_protection_is_curve)

    # A follow-on segment carries a different PSD speed limit. The curve
    # protection must NOT end because the curve cap comes from geometry.
    self.update(curve_target, segment(4, parent_id=3, length=500, street_category=3),
                position(3, 480), speed_attribute(4, 16))
    self.update(curve_target, psd_05=position(4, 500), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, curve_target)

  def test_curve_protection_predicative_type_is_curve(self):
    self.update(100, segment(2, length=200, street_category=5), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 23))
    self.update(100, segment(3, parent_id=2, length=200, street_category=3,
                             curvature_begin=50, curvature_end=50), position(2, 30))
    self.manager.get_speed_limit()
    curve_target = self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH

    self.update(curve_target, psd_05=position(3, 200), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertEqual(self.manager.get_speed_limit_predicative_type(), PSD_TYPE_CURV_SPEED)

  def test_protection_reactivates_after_new_event(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=500), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    # Arrive at first target; VZE still lags.
    self.update(50, psd_05=position(3, 500), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)

    # VZE changes → protection ends (consumed).
    self.update(50, psd_05=position(3, 480), traffic_sign=vze(80))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)
    self.assertTrue(self.manager._speed_protection_consumed)

    # A new, even lower speed-limit event is committed (distance > 0).
    self.update(50, segment(4, parent_id=3, length=500), position(3, 50),
                speed_attribute(4, 8))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    # Arrive at the new target; protection must reactivate.
    self.update(35, psd_05=position(4, 500), traffic_sign=vze(80))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertEqual(self.manager._speed_protection_speed, 35)

  def test_curve_protection_releases_after_distance_exceeded(self):
    self.update(100, segment(2, length=200, street_category=5), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 23))
    self.update(100, segment(3, parent_id=2, length=500, street_category=3,
                             curvature_begin=50, curvature_end=50), position(2, 30))
    self.manager.get_speed_limit()
    curve_target = self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH

    self.update(curve_target, psd_05=position(3, 500), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)

    # Drive beyond the curve-speed-derived protection distance.
    protection_distance = curve_target * CV.KPH_TO_MS * SPEED_SEGMENT_PROTECTION_TIME_S
    remaining = 500 - protection_distance - 1
    self.update(curve_target, psd_05=position(3, max(0, remaining)), traffic_sign=vze(130))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

  def test_consumed_prevents_reactivation_until_new_committed_event(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=500), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    # Arrive at target; VZE lags.
    self.update(50, psd_05=position(3, 500), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)

    # VZE changes → protection ends (consumed).
    self.update(50, psd_05=position(3, 480), traffic_sign=vze(80))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)
    self.assertTrue(self.manager._speed_protection_consumed)

    # Without a new committed event, the protection must not reactivate even
    # if we re-approach the same target segment.
    self.update(50, psd_05=position(3, 470), traffic_sign=vze(80))
    self.manager.get_speed_limit()
    self.assertFalse(self.manager._speed_protection_active)

  def test_predicative_not_set_when_protection_speed_matches_active_limit(self):
    self.add_current_limit()
    self.update(100, segment(3, parent_id=2, length=200), position(2, 30), speed_attribute(3, 11))
    self.manager.get_speed_limit()
    self.assertGreater(self.manager.get_speed_limit_predicative(), 0)

    # Arrive at target; VZE confirms 50 (matches protection speed).
    self.update(50, psd_05=position(3, 200), traffic_sign=vze(50))
    self.manager.get_speed_limit()
    # VZE has changed (100 → 50), protection ends. VZE = 50 = active_limit.
    # Predicative should be NOT_SET (no brake target needed, limit already reached).
    self.assertEqual(self.manager.get_speed_limit_predicative(), NOT_SET)

  def test_lower_psd_speed_limit_within_active_curve(self):
    # A curve is active (cap 80). A follow-on segment within the curve carries
    # a lower PSD speed limit (50). The speed limit must win over the curve cap
    # as the predicative target, and the protection must activate for the speed
    # limit (not the curve) when arriving at the segment.
    self.update(100, segment(2, length=200, street_category=5), position(2, 200))
    self.update(100, psd_05=position(2, 200), psd_06=speed_attribute(2, 23))
    self.assertAlmostEqual(self.manager.get_speed_limit() * CV.MS_TO_KPH, 130)

    # Curve segment with PSD speed limit 100.
    self.update(100, segment(3, parent_id=2, length=500, street_category=3,
                             curvature_begin=50, curvature_end=50), position(2, 30),
                psd_06=speed_attribute(3, 16))
    self.manager.get_speed_limit()
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 80)
    self.assertEqual(self.manager.get_speed_limit_predicative_type(), PSD_TYPE_CURV_SPEED)

    # Follow-on segment: curve continues, but PSD speed limit drops to 50.
    self.update(80, segment(4, parent_id=3, length=500, street_category=3,
                            curvature_begin=50, curvature_end=50), position(3, 100),
                psd_06=speed_attribute(4, 11))
    self.manager.get_speed_limit()
    # The 50 speed limit wins over the 80 curve cap as the predicative target.
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)
    self.assertEqual(self.manager.get_speed_limit_predicative_type(), PSD_TYPE_SPEED_LIMIT)

    # Arrive at the 50 segment while VZE still lags; curve is still active.
    self.update(50, psd_05=position(4, 500), traffic_sign=vze(100))
    self.manager.get_speed_limit()
    self.assertTrue(self.manager._speed_protection_active)
    self.assertFalse(self.manager._speed_protection_is_curve)
    self.assertEqual(self.manager._speed_protection_speed, 50)
    # Predicative output is held at 50.
    self.assertAlmostEqual(self.manager.get_speed_limit_predicative() * CV.MS_TO_KPH, 50)


if __name__ == "__main__":
  unittest.main()
