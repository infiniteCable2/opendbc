import math
import time
from dataclasses import dataclass, field

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import ISO_LATERAL_ACCEL
from opendbc.car.volkswagen.values import VolkswagenFlags


NOT_SET = 0
SPEED_SUGGESTED_MAX_HIGHWAY_GER_KPH = 130  # 130 kph in Germany
STREET_TYPE_URBAN = 1
STREET_TYPE_NONURBAN = 2
STREET_TYPE_HIGHWAY = 3
SPEED_LIMIT_UNLIMITED_VZE_KPH = 520
DECELERATION_PREDICATIVE = 1.0
PSD_TYPE_SPEED_LIMIT = 1
PSD_TYPE_CURV_SPEED = 2
PSD_UNIT_KPH = 0
PSD_UNIT_MPH = 1
MAX_PSD_SEGMENTS = 256

SegmentKey = tuple[int, int]


@dataclass
class PsdSegment:
  key: SegmentKey
  segment_id: int
  incarnation: int
  parent_id: int
  parent_key: SegmentKey | None
  length: float
  curvature_begin: float
  curvature_end: float
  street_type: int
  on_ramp_exit: bool
  likely: bool
  straight: bool
  identity_id: int
  branch_direction: int
  branch_angle: float
  complete: bool
  created_sequence: int
  structural_signature: tuple
  speed_raw: float = NOT_SET
  speed: float = NOT_SET
  speed_offset: float = 0.0
  speed_quality: bool = False
  children: set[SegmentKey] = field(default_factory=set)


@dataclass(frozen=True)
class RouteEvent:
  segment_key: SegmentKey
  offset: float
  end_offset: float
  speed: float
  event_type: int
  route_revision: int

  @property
  def identity(self):
    return (self.route_revision, self.segment_key, self.offset, self.end_offset, self.speed, self.event_type)


class SpeedLimitManager:
  """Fuse observed signs with a persistent VW PSD route graph.

  PSD segment IDs are rolling 6-bit identifiers, so every stored node is qualified
  by a local incarnation. Predictive targets are tied to route-event identities and
  are invalidated by topology/progress, never retained by a wall-clock timeout.
  """

  def __init__(self, car_params, speed_limit_max_kph=SPEED_SUGGESTED_MAX_HIGHWAY_GER_KPH,
               predicative=False, predicative_speed_limit=False, predicative_curve=False):
    self.CP = car_params
    self.v_limit_psd = NOT_SET
    self.v_limit_psd_next = NOT_SET
    self.v_limit_psd_legal = NOT_SET
    self.v_limit_psd_next_type = NOT_SET
    self.v_limit_vze = NOT_SET
    self.v_limit_speed_unit_psd = PSD_UNIT_KPH
    self.v_limit_vze_sanity_error = False
    self.v_limit_output_last = NOT_SET
    self.v_limit_max = speed_limit_max_kph
    self.predicative = predicative
    self.predicative_speed_limit = predicative_speed_limit
    self.predicative_curve = predicative_curve
    self.v_limit_changed = False

    self.predicative_segments: dict[SegmentKey, PsdSegment] = {}
    self._segments_by_id: dict[int, set[SegmentKey]] = {}
    self._active_by_id: dict[int, SegmentKey] = {}
    self._incarnation_by_id: dict[int, int] = {}
    self._pending_speed_attributes: dict[int, tuple[float, float, bool, int]] = {}
    self._legal_limits: dict[int, float] = {}
    self._legal_raw_limits: dict[int, float] = {}
    self._sequence = 0
    self._graph_revision = 0
    self._last_psd_04_payload = None
    self._last_psd_06_speed_payload = None

    self._current_key: SegmentKey | None = None
    self._current_id = NOT_SET
    self._current_remaining = 0.0
    self._current_valid = False
    self._active_map_limit = NOT_SET
    self._map_mismatch_keys: set[SegmentKey] = set()
    self.current_predicative_segment = {
      "ID": NOT_SET, "Length": NOT_SET, "Speed": NOT_SET,
      "StreetType": NOT_SET, "OnRampExit": False,
    }

    self._route: tuple[SegmentKey, ...] = ()
    self._route_index: dict[SegmentKey, int] = {}
    self._route_distance_to_start: dict[SegmentKey, float] = {}
    self._route_graph_revision = -1
    self._route_revision = 0
    self._events: tuple[RouteEvent, ...] = ()
    self._committed_event_identity = None
    self._committed_event_distance = math.inf

  def _reset_predicative(self):
    self.v_limit_psd_next = NOT_SET
    self.v_limit_psd_next_type = NOT_SET
    self._committed_event_identity = None
    self._committed_event_distance = math.inf

  def enable_predicative_speed_limit(self, predicative=False, reaction_to_speed_limits=False, reaction_to_curves=False):
    if self.predicative == predicative and self.predicative_speed_limit == reaction_to_speed_limits and self.predicative_curve == reaction_to_curves:
      return

    if not predicative or (not reaction_to_speed_limits and not reaction_to_curves):
      self._reset_predicative()
      self.predicative = False
    else:
      self.predicative = True

    if (not reaction_to_speed_limits and self.predicative_speed_limit) or (not reaction_to_curves and self.predicative_curve):
      self._reset_predicative()

    self.predicative_speed_limit = reaction_to_speed_limits
    self.predicative_curve = reaction_to_curves
    self._route_graph_revision = -1

  def update(self, current_speed_ms, psd_04, psd_05, psd_06, vze, raining, time_car):
    self._sequence += 1

    if psd_06:
      self._receive_speed_unit_psd(psd_06)

    if psd_04:
      self._ingest_segment_psd(psd_04)

    if psd_06:
      self._receive_speed_attribute_psd(psd_06, raining, time_car)
      self._receive_speed_limit_psd_legal(psd_06)

    if psd_05:
      self._receive_current_segment_psd(psd_05)
    else:
      self._current_valid = False

    self._refresh_current_segment()
    self._update_active_map_limit()

    if vze and self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      self._receive_speed_limit_vze_meb(vze)

    self._update_map_mismatch()
    self._get_speed_limit_psd()
    self._update_legal_limit()

    if self.predicative:
      self._get_speed_limit_psd_next(current_speed_ms)
    else:
      self._reset_predicative()

  def get_speed_limit_predicative(self):
    active_limit = self.v_limit_output_last
    v_limit_output = self.v_limit_psd_next if (self.predicative and self.v_limit_psd_next != NOT_SET and
                                                active_limit != NOT_SET and self.v_limit_psd_next < active_limit) else NOT_SET
    return v_limit_output * CV.KPH_TO_MS

  def get_speed_limit_predicative_type(self):
    return self.v_limit_psd_next_type if self.v_limit_psd_next != NOT_SET else NOT_SET

  def get_speed_limit(self):
    candidates = (
      self.v_limit_vze if not self.v_limit_vze_sanity_error else NOT_SET,
      self.v_limit_psd,
      self.v_limit_psd_legal,
    )
    v_limit_output = next((v for v in candidates if v != NOT_SET), NOT_SET)
    if v_limit_output > self.v_limit_max:
      v_limit_output = self.v_limit_max

    self.v_limit_changed = self.v_limit_output_last != v_limit_output
    self.v_limit_output_last = v_limit_output
    return v_limit_output * CV.KPH_TO_MS

  def _receive_speed_unit_psd(self, psd_06):
    if psd_06.get("PSD_06_Mux") == 0 and psd_06.get("PSD_Sys_Segment_ID", NOT_SET) > 1:
      unit = psd_06.get("PSD_Sys_Geschwindigkeit_Einheit", PSD_UNIT_KPH)
      if unit in (PSD_UNIT_KPH, PSD_UNIT_MPH) and unit != self.v_limit_speed_unit_psd:
        self.v_limit_speed_unit_psd = unit
        self._active_map_limit = NOT_SET
        for seg in self.predicative_segments.values():
          if seg.speed_quality:
            seg.speed = self._convert_raw_speed_psd(seg.speed_raw, seg.street_type)
            seg.speed_quality = seg.speed != NOT_SET
        for street_type, raw_speed in self._legal_raw_limits.items():
          self._legal_limits[street_type] = self._convert_raw_speed_psd(raw_speed, street_type)
        self._graph_revision += 1

  def _convert_raw_speed_psd(self, raw_speed, street_type):
    speed = NOT_SET
    if self.v_limit_speed_unit_psd == PSD_UNIT_KPH:
      if 0 < raw_speed < 11:
        speed = (raw_speed - 1) * 5
      elif 11 <= raw_speed < 23:
        speed = 50 + (raw_speed - 11) * 10
      elif raw_speed == 23 and street_type == STREET_TYPE_HIGHWAY:
        speed = self.v_limit_max
    elif self.v_limit_speed_unit_psd == PSD_UNIT_MPH:
      if 3 < raw_speed < 18:
        speed = (5 * (raw_speed - 3)) * CV.MPH_TO_KPH
      elif 18 <= raw_speed < 23:
        speed = ((5 * (raw_speed - 3)) + 10) * CV.MPH_TO_KPH
      elif raw_speed == 23 and street_type == STREET_TYPE_HIGHWAY:
        speed = self.v_limit_max
    return speed

  def _receive_speed_limit_vze_meb(self, vze):
    raw_limit = vze.get("VZE_Verkehrszeichen_1", NOT_SET)
    if raw_limit <= NOT_SET or vze.get("VZE_Verkehrszeichen_1_Typ", 0) != 0 or raw_limit >= SPEED_LIMIT_UNLIMITED_VZE_KPH:
      self.v_limit_vze = NOT_SET
      self.v_limit_vze_sanity_error = False
      return

    v_limit_vze = raw_limit
    if vze.get("VZE_Anzeigemodus") == 1 or self.v_limit_speed_unit_psd == PSD_UNIT_MPH:
      v_limit_vze *= CV.MPH_TO_KPH

    # A plausible recognized sign is authoritative, even when it differs greatly
    # from the map. Values above the VW unlimited-sign encoding are not limits.
    self.v_limit_vze_sanity_error = not (0 < v_limit_vze < SPEED_LIMIT_UNLIMITED_VZE_KPH)
    self.v_limit_vze = NOT_SET if self.v_limit_vze_sanity_error else v_limit_vze

  def _segment_payload(self, psd_04):
    names = (
      "PSD_Segment_ID", "PSD_Vorgaenger_Segment_ID", "PSD_Segmentlaenge", "PSD_Strassenkategorie",
      "PSD_Endkruemmung", "PSD_Endkruemmung_Vorz", "PSD_Idenditaets_ID", "PSD_ADAS_Qualitaet",
      "PSD_wahrscheinlichster_Pfad", "PSD_Geradester_Pfad", "PSD_Bebauung", "PSD_Segment_Komplett",
      "PSD_Rampe", "PSD_Anfangskruemmung", "PSD_Anfangskruemmung_Vorz", "PSD_Abzweigerichtung",
      "PSD_Abzweigewinkel",
    )
    return tuple(psd_04.get(name) for name in names)

  def _segment_signature(self, psd_04):
    return (
      psd_04.get("PSD_Vorgaenger_Segment_ID", NOT_SET), psd_04.get("PSD_Segmentlaenge", 0),
      psd_04.get("PSD_Strassenkategorie", 0), psd_04.get("PSD_Bebauung", 0), psd_04.get("PSD_Rampe", 0),
      psd_04.get("PSD_Anfangskruemmung", 255), psd_04.get("PSD_Anfangskruemmung_Vorz", 0),
      psd_04.get("PSD_Endkruemmung", 255), psd_04.get("PSD_Endkruemmung_Vorz", 0),
      psd_04.get("PSD_Idenditaets_ID", NOT_SET), psd_04.get("PSD_Abzweigerichtung", 0),
      psd_04.get("PSD_Abzweigewinkel", 0),
    )

  def _ingest_segment_psd(self, psd_04):
    payload = self._segment_payload(psd_04)
    if payload == self._last_psd_04_payload:
      return
    self._last_psd_04_payload = payload

    if psd_04.get("PSD_ADAS_Qualitaet") != 1:
      return
    segment_id = psd_04.get("PSD_Segment_ID", NOT_SET)
    if segment_id <= 1:
      return

    signature = self._segment_signature(psd_04)
    active_key = self._active_by_id.get(segment_id)
    seg = self.predicative_segments.get(active_key) if active_key is not None else None
    parent_id = psd_04.get("PSD_Vorgaenger_Segment_ID", NOT_SET)
    expected_parent_key = self._active_by_id.get(parent_id) if parent_id > 1 else None
    parent_incarnation_changed = seg is not None and seg.parent_key != expected_parent_key
    if seg is None or seg.structural_signature != signature or parent_incarnation_changed:
      incarnation = self._incarnation_by_id.get(segment_id, 0) + 1
      self._incarnation_by_id[segment_id] = incarnation
      key = (segment_id, incarnation)
      parent_key = expected_parent_key
      seg = PsdSegment(
        key=key,
        segment_id=segment_id,
        incarnation=incarnation,
        parent_id=parent_id,
        parent_key=parent_key,
        length=max(0.0, float(psd_04.get("PSD_Segmentlaenge", 0))),
        curvature_begin=self._get_segment_curvature_psd(psd_04.get("PSD_Anfangskruemmung", 255), psd_04.get("PSD_Anfangskruemmung_Vorz", 0)),
        curvature_end=self._get_segment_curvature_psd(psd_04.get("PSD_Endkruemmung", 255), psd_04.get("PSD_Endkruemmung_Vorz", 0)),
        street_type=self._get_street_type(psd_04.get("PSD_Strassenkategorie", 0), psd_04.get("PSD_Bebauung", 0)),
        on_ramp_exit=psd_04.get("PSD_Rampe", 0) in (1, 2),
        likely=bool(psd_04.get("PSD_wahrscheinlichster_Pfad", 0)),
        straight=bool(psd_04.get("PSD_Geradester_Pfad", 0)),
        identity_id=psd_04.get("PSD_Idenditaets_ID", NOT_SET),
        branch_direction=psd_04.get("PSD_Abzweigerichtung", 0),
        branch_angle=float(psd_04.get("PSD_Abzweigewinkel", 0)),
        complete=bool(psd_04.get("PSD_Segment_Komplett", 0)),
        created_sequence=self._sequence,
        structural_signature=signature,
      )
      self.predicative_segments[key] = seg
      self._segments_by_id.setdefault(segment_id, set()).add(key)
      self._active_by_id[segment_id] = key
      if parent_key in self.predicative_segments:
        self.predicative_segments[parent_key].children.add(key)
      # PSD is multiplexed and a child can be seen before its parent. Resolve
      # only unparented active children; already linked incarnations stay intact.
      current_horizon = self._reachable_from(self._current_key) if self._current_key is not None else set()
      for child in self.predicative_segments.values():
        if (child.key != key and child.key not in current_horizon and child.parent_id == segment_id and child.parent_key is None and
            self._active_by_id.get(child.segment_id) == child.key):
          child.parent_key = key
          seg.children.add(child.key)
      self._apply_pending_speed_attribute(seg)
      self._graph_revision += 1
      self._resolve_current_key()
      self._bound_graph()
    else:
      annotations = (seg.likely, seg.straight, seg.complete)
      seg.likely = bool(psd_04.get("PSD_wahrscheinlichster_Pfad", 0))
      seg.straight = bool(psd_04.get("PSD_Geradester_Pfad", 0))
      seg.complete = bool(psd_04.get("PSD_Segment_Komplett", 0))
      if annotations != (seg.likely, seg.straight, seg.complete):
        self._graph_revision += 1

  def _receive_current_segment_psd(self, psd_05):
    unique = psd_05.get("PSD_Pos_Standort_Eindeutig") == 1
    segment_id = psd_05.get("PSD_Pos_Segment_ID", NOT_SET)
    remaining = float(psd_05.get("PSD_Pos_Segmentlaenge", 0))
    if not unique or segment_id <= 1 or remaining < 0:
      self._current_valid = False
      self._reset_predicative()
      return

    changed = segment_id != self._current_id
    self._current_id = segment_id
    self._current_remaining = remaining
    self._current_valid = True
    if changed:
      previous_key = self._current_key
      self._current_key = self._select_current_key(segment_id, previous_key)
      self._graph_revision += 1
      if self._current_key is not None:
        self._prune_to_current_horizon()
    elif self._current_key is None:
      self._resolve_current_key()

  def _select_current_key(self, segment_id, previous_key):
    candidates = list(self._segments_by_id.get(segment_id, ()))
    if not candidates:
      return None
    if previous_key in self.predicative_segments:
      direct = [key for key in candidates if self.predicative_segments[key].parent_key == previous_key]
      if len(direct) == 1:
        return direct[0]
      likely = [key for key in direct if self.predicative_segments[key].likely]
      if len(likely) == 1:
        return likely[0]
    return max(candidates, key=lambda key: self.predicative_segments[key].created_sequence)

  def _resolve_current_key(self):
    if not self._current_valid or self._current_id <= 1:
      return
    if self._current_key is not None and self._current_key[0] == self._current_id:
      return
    self._current_key = self._select_current_key(self._current_id, self._current_key)
    if self._current_key is not None:
      self._graph_revision += 1
      self._prune_to_current_horizon()

  def _reachable_from(self, start_key):
    reachable = set()
    stack = [start_key]
    while stack:
      key = stack.pop()
      if key in reachable or key not in self.predicative_segments:
        continue
      reachable.add(key)
      stack.extend(self.predicative_segments[key].children)
    return reachable

  def _remove_segment(self, key):
    seg = self.predicative_segments.pop(key, None)
    if seg is None:
      return
    if seg.parent_key in self.predicative_segments:
      self.predicative_segments[seg.parent_key].children.discard(key)
    for child_key in seg.children:
      if child_key in self.predicative_segments:
        self.predicative_segments[child_key].parent_key = None
    id_keys = self._segments_by_id.get(seg.segment_id)
    if id_keys is not None:
      id_keys.discard(key)
      if not id_keys:
        self._segments_by_id.pop(seg.segment_id, None)
        self._active_by_id.pop(seg.segment_id, None)
      elif self._active_by_id.get(seg.segment_id) == key:
        self._active_by_id[seg.segment_id] = max(id_keys, key=lambda item: self.predicative_segments[item].created_sequence)
    self._map_mismatch_keys.discard(key)

  def _prune_to_current_horizon(self):
    reachable = self._reachable_from(self._current_key)
    for key in tuple(self.predicative_segments):
      if key not in reachable:
        self._remove_segment(key)
    self._graph_revision += 1

  def _bound_graph(self):
    if len(self.predicative_segments) <= MAX_PSD_SEGMENTS:
      return
    protected = self._reachable_from(self._current_key) if self._current_key is not None else set()
    removable = sorted((seg for seg in self.predicative_segments.values() if seg.key not in protected),
                       key=lambda seg: seg.created_sequence)
    for seg in removable[:max(0, len(self.predicative_segments) - MAX_PSD_SEGMENTS)]:
      self._remove_segment(seg.key)
    self._graph_revision += 1

  def _get_segment_curvature_psd(self, psd_curvature, psd_sign):
    """Decode the ADASIS v2 piecewise inverse-curvature code to 1/m."""
    if not 0 <= psd_curvature <= 255:
      return 0.0
    magnitude_code = 255 - psd_curvature
    if magnitude_code <= 64:
      curvature = magnitude_code / 100000.0
    elif magnitude_code <= 128:
      curvature = 2 * (magnitude_code - 32) / 100000.0
    elif magnitude_code <= 192:
      curvature = 4 * (magnitude_code - 80) / 100000.0
    else:
      curvature = 8 * (magnitude_code - 136) / 100000.0
    return -curvature if psd_sign == 1 else curvature

  def _calculate_curve_speed(self, curvature):
    if abs(curvature) < 1e-12:
      return NOT_SET
    curv_speed_ms = math.sqrt(ISO_LATERAL_ACCEL / abs(curvature))
    if self.v_limit_speed_unit_psd == PSD_UNIT_MPH:
      return int((curv_speed_ms * CV.MS_TO_MPH) // 5 * 5) * CV.MPH_TO_KPH
    return int((curv_speed_ms * CV.MS_TO_KPH) // 5 * 5)

  def _refresh_current_segment(self):
    seg = self.predicative_segments.get(self._current_key)
    self.current_predicative_segment["ID"] = self._current_id if self._current_valid else NOT_SET
    self.current_predicative_segment["Length"] = self._current_remaining if self._current_valid else NOT_SET
    self.current_predicative_segment["Speed"] = self._active_map_limit
    self.current_predicative_segment["StreetType"] = seg.street_type if seg is not None else NOT_SET
    self.current_predicative_segment["OnRampExit"] = seg.on_ramp_exit if seg is not None else False

  def _speed_attribute_payload(self, psd_06, raining, time_car):
    names = (
      "PSD_06_Mux", "PSD_Ges_Segment_ID", "PSD_Ges_Offset", "PSD_Ges_Geschwindigkeit", "PSD_Ges_Typ",
      "PSD_Ges_Geschwindigkeit_Witter", "PSD_Ges_Geschwindigkeit_Tag_Anf", "PSD_Ges_Geschwindigkeit_Tag_Ende",
      "PSD_Ges_Geschwindigkeit_Std_Anf", "PSD_Ges_Geschwindigkeit_Std_Ende", "PSD_Ges_Gesetzlich_Kategorie",
    )
    local_time = self._get_time_from_vw_datetime(time_car)
    return (*tuple(psd_06.get(name) for name in names), bool(raining), local_time.tm_wday, local_time.tm_hour)

  def _receive_speed_attribute_psd(self, psd_06, raining, time_car):
    if psd_06.get("PSD_06_Mux") != 2 or psd_06.get("PSD_Ges_Typ") != 1 or psd_06.get("PSD_Ges_Gesetzlich_Kategorie") != 0:
      return
    segment_id = psd_06.get("PSD_Ges_Segment_ID", NOT_SET)
    if segment_id <= 1:
      return
    payload = self._speed_attribute_payload(psd_06, raining, time_car)
    if payload == self._last_psd_06_speed_payload:
      return
    self._last_psd_06_speed_payload = payload

    valid = self._speed_limit_is_valid_now_psd(psd_06, raining, time_car)
    raw_speed = psd_06.get("PSD_Ges_Geschwindigkeit", NOT_SET) if valid else NOT_SET
    offset = max(0.0, float(psd_06.get("PSD_Ges_Offset", 0)))
    key = self._active_by_id.get(segment_id)
    seg = self.predicative_segments.get(key)
    if seg is None:
      expected_incarnation = self._incarnation_by_id.get(segment_id, 0) + 1
      self._pending_speed_attributes[segment_id] = (raw_speed, offset, valid, expected_incarnation)
      return
    self._set_speed_attribute(seg, raw_speed, offset, valid)

  def _set_speed_attribute(self, seg, raw_speed, offset, valid):
    speed = self._convert_raw_speed_psd(raw_speed, seg.street_type) if valid else NOT_SET
    values = (seg.speed_raw, seg.speed, seg.speed_offset, seg.speed_quality)
    seg.speed_raw = raw_speed
    seg.speed = speed
    seg.speed_offset = min(offset, seg.length) if seg.length > 0 else offset
    seg.speed_quality = valid and speed != NOT_SET
    if values != (seg.speed_raw, seg.speed, seg.speed_offset, seg.speed_quality):
      self._graph_revision += 1

  def _apply_pending_speed_attribute(self, seg):
    attribute = self._pending_speed_attributes.pop(seg.segment_id, None)
    if attribute is not None and attribute[3] == seg.incarnation:
      self._set_speed_attribute(seg, *attribute[:3])

  def _update_active_map_limit(self):
    if not self._current_valid or self._current_key is None:
      self.v_limit_psd = NOT_SET
      return
    seg = self.predicative_segments.get(self._current_key)
    if seg is None:
      self.v_limit_psd = NOT_SET
      return
    progress = max(0.0, seg.length - self._current_remaining)
    if seg.speed_quality and progress >= seg.speed_offset:
      self._active_map_limit = seg.speed
      if self.v_limit_vze != NOT_SET and math.isclose(self.v_limit_vze, seg.speed, abs_tol=1.0):
        self._map_mismatch_keys.discard(self._current_key)
    self.current_predicative_segment["Speed"] = self._active_map_limit

  def _update_map_mismatch(self):
    if not self._current_valid or self._current_key is None or self.v_limit_vze == NOT_SET:
      return
    seg = self.predicative_segments.get(self._current_key)
    legal_limit = self._legal_limits.get(seg.street_type, NOT_SET) if seg is not None else NOT_SET
    map_reference = self._active_map_limit or legal_limit
    if map_reference == NOT_SET:
      return
    if math.isclose(self.v_limit_vze, map_reference, abs_tol=1.0):
      self._map_mismatch_keys.discard(self._current_key)
    else:
      self._map_mismatch_keys.add(self._current_key)

  def _get_speed_limit_psd(self):
    if (not self._current_valid or self._current_key is None or self._active_map_limit == NOT_SET or
        self._current_key in self._map_mismatch_keys):
      self.v_limit_psd = NOT_SET
    else:
      self.v_limit_psd = self._active_map_limit

  def _select_child(self, seg):
    children = [key for key in seg.children if key in self.predicative_segments]
    if len(children) == 1:
      return children[0]
    likely = [key for key in children if self.predicative_segments[key].likely]
    return likely[0] if len(likely) == 1 else None

  def _ensure_route_cache(self):
    if self._route_graph_revision == self._graph_revision:
      return
    route = []
    key = self._current_key if self._current_valid else None
    visited = set()
    while key is not None and key not in visited and key in self.predicative_segments:
      visited.add(key)
      route.append(key)
      key = self._select_child(self.predicative_segments[key])
    new_route = tuple(route)
    if new_route != self._route:
      # Advancing to a suffix is progress on the same route, not a path
      # revision. This keeps a committed curve event alive through entry.
      progressed_on_route = bool(new_route and len(new_route) <= len(self._route) and self._route[-len(new_route):] == new_route)
      extended_same_route = bool(self._route and len(new_route) >= len(self._route) and new_route[:len(self._route)] == self._route)
      if not progressed_on_route and not extended_same_route:
        self._route_revision += 1
      self._route = new_route
    self._route_index = {route_key: index for index, route_key in enumerate(self._route)}
    distance = 0.0
    self._route_distance_to_start = {}
    for index, route_key in enumerate(self._route):
      self._route_distance_to_start[route_key] = distance
      if index > 0:
        distance += self.predicative_segments[route_key].length
    self._events = self._build_route_events()
    self._route_graph_revision = self._graph_revision
    identities = {event.identity for event in self._events}
    if self._committed_event_identity not in identities:
      self._committed_event_identity = None
      self._committed_event_distance = math.inf

  def _curve_speed_for_segment(self, seg):
    speeds = [speed for speed in (self._calculate_curve_speed(seg.curvature_begin), self._calculate_curve_speed(seg.curvature_end))
              if speed != NOT_SET]
    return min(speeds) if speeds else NOT_SET

  def _speed_limit_curve_allowed(self, seg, speed_curve):
    # Highway geometry remains disabled except for explicitly marked ramps.
    # The selected future ramp is allowed before the car enters it so braking
    # can actually be predictive.
    street_type_allowed = seg.street_type == STREET_TYPE_NONURBAN or (seg.street_type == STREET_TYPE_HIGHWAY and seg.on_ramp_exit)
    return street_type_allowed and speed_curve > 0

  def _build_route_events(self):
    events = []
    for key in self._route:
      seg = self.predicative_segments[key]
      if self.predicative_speed_limit and seg.speed_quality:
        events.append(RouteEvent(key, seg.speed_offset, seg.speed_offset, seg.speed, PSD_TYPE_SPEED_LIMIT, self._route_revision))
      if self.predicative_curve:
        curve_speed = self._curve_speed_for_segment(seg)
        if curve_speed != NOT_SET and self._speed_limit_curve_allowed(seg, curve_speed):
          events.append(RouteEvent(key, 0.0, seg.length, curve_speed, PSD_TYPE_CURV_SPEED, self._route_revision))
    return tuple(events)

  def _event_distance(self, event):
    event_index = self._route_index.get(event.segment_key)
    if event_index is None:
      return None, False
    current_seg = self.predicative_segments[self._route[0]]
    current_progress = max(0.0, current_seg.length - self._current_remaining)
    if event_index == 0:
      if event.event_type == PSD_TYPE_CURV_SPEED and event.offset <= current_progress <= event.end_offset:
        return 0.0, True
      return event.offset - current_progress, False

    return self._current_remaining + self._route_distance_to_start[event.segment_key] + event.offset, False

  def _get_speed_limit_psd_next(self, current_speed_ms):
    self.v_limit_psd_next = NOT_SET
    self.v_limit_psd_next_type = NOT_SET
    if not self._current_valid or self._current_key is None or current_speed_ms <= 0:
      self._reset_predicative()
      return

    self._ensure_route_cache()
    candidates = []
    committed = None
    for event in self._events:
      distance, active_curve = self._event_distance(event)
      if distance is None:
        continue
      is_committed = event.identity == self._committed_event_identity
      if is_committed:
        committed = (event, distance, active_curve)
      if event.event_type == PSD_TYPE_SPEED_LIMIT and distance <= 0:
        continue
      if event.speed * CV.KPH_TO_MS >= current_speed_ms and not active_curve:
        continue
      brake_distance = max(0.0, (current_speed_ms ** 2 - (event.speed * CV.KPH_TO_MS) ** 2) / (2 * DECELERATION_PREDICATIVE))
      if active_curve or 0 <= distance <= brake_distance:
        candidates.append((event, max(0.0, distance)))

    if committed is not None:
      event, distance, active_curve = committed
      if active_curve or distance >= 0:
        distance = min(max(0.0, distance), self._committed_event_distance)
        candidates.append((event, distance))
      else:
        self._committed_event_identity = None
        self._committed_event_distance = math.inf

    if not candidates:
      return

    event, distance = min(candidates, key=lambda candidate: (candidate[0].speed, candidate[1], candidate[0].event_type))
    self._committed_event_identity = event.identity
    self._committed_event_distance = distance
    self.v_limit_psd_next = event.speed
    self.v_limit_psd_next_type = event.event_type

  def _get_time_from_vw_datetime(self, time_car):
    if time_car:
      try:
        date_time = (time_car["UH_Jahr"], time_car["UH_Monat"], time_car["UH_Tag"], time_car["UH_Stunde"],
                     time_car["UH_Minute"], time_car["UH_Sekunde"], 0, 0, -1)
        return time.localtime(time.mktime(date_time))
      except (KeyError, OverflowError, TypeError, ValueError):
        pass
    return time.localtime()

  def _speed_limit_is_valid_now_psd(self, psd_06, raining, time_car):
    local_time = self._get_time_from_vw_datetime(time_car)
    day_start = psd_06.get("PSD_Ges_Geschwindigkeit_Tag_Anf", 0)
    day_end = psd_06.get("PSD_Ges_Geschwindigkeit_Tag_Ende", 0)
    now_weekday = local_time.tm_wday + 1
    if 1 <= day_start <= 7 and 1 <= day_end <= 7:
      is_valid_by_day = day_start <= now_weekday <= day_end if day_start <= day_end else now_weekday >= day_start or now_weekday <= day_end
    else:
      is_valid_by_day = True

    hour_start = psd_06.get("PSD_Ges_Geschwindigkeit_Std_Anf", 25)
    hour_end = psd_06.get("PSD_Ges_Geschwindigkeit_Std_Ende", 25)
    if hour_start != 25 and hour_end != 25:
      is_valid_by_time = (hour_start <= local_time.tm_hour < hour_end if hour_start <= hour_end
                          else local_time.tm_hour >= hour_start or local_time.tm_hour < hour_end)
    else:
      is_valid_by_time = True

    weather_condition = psd_06.get("PSD_Ges_Geschwindigkeit_Witter", 0)
    is_valid_by_weather = weather_condition == 0 or (raining and weather_condition == 1)
    return is_valid_by_day and is_valid_by_time and is_valid_by_weather

  def _get_street_type(self, strassenkategorie, bebauung):
    if strassenkategorie == 1:
      return STREET_TYPE_URBAN
    if strassenkategorie in (2, 3, 4):
      return STREET_TYPE_URBAN if bebauung == 1 else STREET_TYPE_NONURBAN
    if strassenkategorie == 5:
      return STREET_TYPE_HIGHWAY
    return NOT_SET

  def _receive_speed_limit_psd_legal(self, psd_06):
    if psd_06.get("PSD_06_Mux") != 2 or psd_06.get("PSD_Ges_Typ") != 2:
      return
    street_type = psd_06.get("PSD_Ges_Gesetzlich_Kategorie", NOT_SET)
    if street_type in (STREET_TYPE_URBAN, STREET_TYPE_NONURBAN, STREET_TYPE_HIGHWAY):
      raw_speed = psd_06.get("PSD_Ges_Geschwindigkeit", NOT_SET)
      speed = self._convert_raw_speed_psd(raw_speed, street_type)
      if speed == NOT_SET:
        self._legal_limits.pop(street_type, None)
        self._legal_raw_limits.pop(street_type, None)
      else:
        self._legal_limits[street_type] = speed
        self._legal_raw_limits[street_type] = raw_speed

  def _update_legal_limit(self):
    seg = self.predicative_segments.get(self._current_key)
    if seg is None or not self._current_valid or self._current_key in self._map_mismatch_keys:
      self.v_limit_psd_legal = NOT_SET
    else:
      self.v_limit_psd_legal = self._legal_limits.get(seg.street_type, NOT_SET)
