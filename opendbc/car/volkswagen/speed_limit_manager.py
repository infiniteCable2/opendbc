import time
import math

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import VolkswagenFlags

NOT_SET = 0
SPEED_SUGGESTED_MAX_HIGHWAY_GER_KPH = 130 # 130 kph in germany
STREET_TYPE_URBAN = 1
STREET_TYPE_NONURBAN = 2
STREET_TYPE_HIGHWAY = 3
SANITY_CHECK_DIFF_PERCENT_LOWER = 30
SPEED_LIMIT_UNLIMITED_VZE_KPH = int(round(144 * CV.MS_TO_KPH))
DECELERATION_PREDICATIVE = 1.0
SEGMENT_DECAY = 10
PSD_UNIT_KPH = 0
PSD_UNIT_MPH = 1


class SpeedLimitManager:
  def __init__(self, car_params, speed_limit_max_kph=SPEED_SUGGESTED_MAX_HIGHWAY_GER_KPH, predicative=False):
    self.CP = car_params
    self.v_limit_psd = NOT_SET
    self.v_limit_psd_next = NOT_SET
    self.v_limit_psd_legal = NOT_SET
    self.v_limit_vze = NOT_SET
    self.v_limit_speed_unit_psd = PSD_UNIT_KPH
    self.v_limit_vze_sanity_error = False
    self.v_limit_output_last = NOT_SET
    self.v_limit_max = speed_limit_max_kph
    self.predicative = predicative
    self.predicative_segments = {}
    self.current_predicative_segment = {"ID": NOT_SET, "Length": NOT_SET, "Speed": NOT_SET, "StreetType": NOT_SET}
    self.v_limit_psd_next_last_timestamp = 0
    self.v_limit_psd_next_last = NOT_SET
    self.v_limit_psd_next_decay_time = NOT_SET
    self.v_limit_changed = False

  def update(self, current_speed_ms, psd_04, psd_05, psd_06, vze, raining):
    # try reading speed form traffic sign recognition
    if vze and self.CP.flags & VolkswagenFlags.MEB:
      self._receive_speed_limit_vze_meb(vze)

    # try reading speed from predicative street data
    if psd_04 and psd_05 and psd_06:
      self._receive_speed_unit_psd(psd_06)
      self._receive_current_segment_psd(psd_05)
      self._refresh_current_segment()
      self._build_predicative_segments(psd_04, psd_06, raining)
      self._receive_speed_limit_psd_legal(psd_06)
      self._get_speed_limit_psd()
      self._get_speed_limit_psd_next(current_speed_ms)

  def get_speed_limit_predicative(self):
    v_limit_output = self.v_limit_psd_next if self.predicative and self.v_limit_psd_next != NOT_SET and self.v_limit_psd_next < self.v_limit_output_last else NOT_SET
    
    return v_limit_output * CV.KPH_TO_MS

  def get_speed_limit(self):
    candidates = {
      "vze": self.v_limit_vze if self.v_limit_vze != NOT_SET and not self.v_limit_vze_sanity_error else NOT_SET,
      "psd": self.v_limit_psd if self.v_limit_psd != NOT_SET else NOT_SET,
      "legal": self.v_limit_psd_legal
    }

    v_limit_output = NOT_SET
    for source in ["vze", "psd", "legal"]:
      v = candidates[source]
      if v != NOT_SET:
        v_limit_output = v
        break
  
    if v_limit_output > self.v_limit_max:
      v_limit_output = self.v_limit_max
  
    self.v_limit_vze_sanity_error = False
    
    self.v_limit_changed = True if self.v_limit_output_last != v_limit_output else False
    self.v_limit_output_last = v_limit_output
  
    return v_limit_output * CV.KPH_TO_MS

  def _speed_limit_vze_sanitiy_check(self, speed_limit_vze_new):
    if self.v_limit_output_last == NOT_SET:
      return

    diff_p = 100 * speed_limit_vze_new / self.v_limit_output_last
    self.v_limit_vze_sanity_error = True if diff_p < SANITY_CHECK_DIFF_PERCENT_LOWER else False
    if speed_limit_vze_new > SPEED_LIMIT_UNLIMITED_VZE_KPH: # unlimited sign detected: use psd logic for setting maximum speed
      self.v_limit_vze_sanity_error = True

  def _receive_speed_unit_psd(self, psd_06):
    # keep it simple for now, the unit is supplied shortly before the corresponding speed limits are supplied for given segment ID
    if psd_06["PSD_06_Mux"] == 0 and psd_06["PSD_Sys_Segment_ID"] != NOT_SET:
      self.v_limit_speed_unit_psd = psd_06["PSD_Sys_Geschwindigkeit_Einheit"]

  def _convert_raw_speed_psd(self, raw_speed, street_type):
    # PSD raw_speed mapping:
    #  KPH: 1–10 -> 0–45 km/h (steps of 5), 11–22 -> 50–160 km/h (steps of 10)
    #  MPH: 1–10 -> 15–65 mph (steps of 5), 11–22 -> 70–120 mph (steps of 5)
    speed = NOT_SET
    
    if self.v_limit_speed_unit_psd == PSD_UNIT_KPH:
      if 0 < raw_speed < 11: # 0 - 45 kph
        speed = (raw_speed - 1) * 5
      elif 11 <= raw_speed < 23: # 50 - 160 kph
        speed = 50 + (raw_speed - 11) * 10
      elif raw_speed == 23: # explicitly no legal speed limit 
        if street_type == STREET_TYPE_HIGHWAY:
          speed = self.v_limit_max

    elif self.v_limit_speed_unit_psd == PSD_UNIT_MPH:
      if 0 < raw_speed < 11:  # 15 - 65 mph
        speed = (10 + raw_speed * 5) * CV.MPH_TO_KPH
      elif 11 <= raw_speed < 23:  # 70 - 120 mph
        speed = (15 + (raw_speed - 10) * 5 ) * CV.MPH_TO_KPH
      elif raw_speed == 23:  # explicitly no legal speed limit
        if street_type == STREET_TYPE_HIGHWAY:
          speed = self.v_limit_max

    return speed

  def _receive_speed_limit_vze_meb(self, vze):
    v_limit_vze = int(round(vze["VZE_Verkehrszeichen_1"])) # main traffic sign
    v_limit_vze = v_limit_vze * CV.MPH_TO_KPH if vze["VZE_Anzeigemodus"] == 1 else v_limit_vze
    self._speed_limit_vze_sanitiy_check(v_limit_vze)
    self.v_limit_vze = v_limit_vze

  def _receive_current_segment_psd(self, psd_05):
    if psd_05["PSD_Pos_Standort_Eindeutig"] == 1 and psd_05["PSD_Pos_Segment_ID"] != NOT_SET:
      self.current_predicative_segment["Length"] = psd_05["PSD_Pos_Segmentlaenge"]
      
      if self.current_predicative_segment["ID"] != psd_05["PSD_Pos_Segment_ID"]:
        self.current_predicative_segment["ID"] = psd_05["PSD_Pos_Segment_ID"]
        self.current_predicative_segment["Speed"] = NOT_SET
        self.current_predicative_segment["StreetType"] = NOT_SET

  def _refresh_current_segment(self):
    current_segment = self.current_predicative_segment["ID"]
    if current_segment != NOT_SET:
      seg = self.predicative_segments.get(current_segment)
      if seg:
        self.current_predicative_segment["Speed"] = self.predicative_segments[current_segment]["Speed"]
        self.current_predicative_segment["StreetType"] = self.predicative_segments[current_segment]["StreetType"]

  def _build_predicative_segments(self, psd_04, psd_06, raining):
    now = time.time()

    # Segment erfassen/aktualisieren
    if (psd_04["PSD_ADAS_Qualitaet"] == 1 and
        psd_04["PSD_wahrscheinlichster_Pfad"] == 1 and
        psd_04["PSD_Segment_ID"] != NOT_SET):

      segment_id = psd_04["PSD_Segment_ID"]
      seg = self.predicative_segments.get(segment_id)
      if seg:
        seg["Length"] = psd_04["PSD_Segmentlaenge"]
        seg["StreetType"] = self._get_street_type(psd_04["PSD_Strassenkategorie"], psd_04["PSD_Bebauung"])
        seg["ID_Prev"] = psd_04["PSD_Vorgaenger_Segment_ID"]
        seg["Timestamp"] = now
      else:
        self.predicative_segments[segment_id] = {
          "ID": segment_id,
          "Length": psd_04["PSD_Segmentlaenge"],
          "StreetType": self._get_street_type(psd_04["PSD_Strassenkategorie"], psd_04["PSD_Bebauung"]),
          "ID_Prev": psd_04["PSD_Vorgaenger_Segment_ID"],
          "Speed": NOT_SET,
          "QualityFlag": False,
          "Timestamp": now
        }

    # Schritt 2: Alte Segmente bereinigen
    current_id = self.current_predicative_segment["ID"]
    if current_id != NOT_SET:
      self.predicative_segments = {
        sid: seg for sid, seg in self.predicative_segments.items()
        if now - seg.get("Timestamp", 0) <= SEGMENT_DECAY
      }

    # Geschwindigkeit setzen (speed limits seen for changed limits only)
    if (psd_06["PSD_06_Mux"] == 2 and
        psd_06["PSD_Ges_Typ"] == 1 and
        psd_06["PSD_Ges_Gesetzlich_Kategorie"] == 0 and
        psd_06["PSD_Ges_Segment_ID"] != NOT_SET):

      raw_speed = psd_06["PSD_Ges_Geschwindigkeit"] if self._speed_limit_is_valid_now_psd(psd_06, raining) else NOT_SET
      segment_id = psd_06["PSD_Ges_Segment_ID"]

      if segment_id in self.predicative_segments:
        speed = self._convert_raw_speed_psd(raw_speed, self.predicative_segments[segment_id]["StreetType"])
        self.predicative_segments[segment_id]["Speed"] = speed
        self.predicative_segments[segment_id]["QualityFlag"] = True

  def _speed_limit_is_valid_now_psd(self, psd_06, raining):
    # by day
    day_start = psd_06["PSD_Ges_Geschwindigkeit_Tag_Anf"]
    day_end = psd_06["PSD_Ges_Geschwindigkeit_Tag_Ende"]
    now_weekday = (time.localtime().tm_wday + 1)  # Python: 0=Montag → PSD: 1=Montag

    if 1 <= day_start <= 7 and 1 <= day_end <= 7:
      if day_start <= day_end:
        is_valid_by_day = day_start <= now_weekday <= day_end
      else:
        is_valid_by_day = now_weekday >= day_start or now_weekday <= day_end
    else:
      is_valid_by_day = True
  
    # by time
    hour_start = psd_06["PSD_Ges_Geschwindigkeit_Std_Anf"]
    hour_end = psd_06["PSD_Ges_Geschwindigkeit_Std_Ende"]
    now_hour = time.localtime().tm_hour
    
    if (hour_start != 25 and hour_end != 25):
      if hour_start <= hour_end:
        is_valid_by_time = hour_start <= now_hour < hour_end
      else:
        is_valid_by_time = now_hour >= hour_start or now_hour < hour_end
    else:
      is_valid_by_time = True

    # by weather condition
    weather_condition = psd_06["PSD_Ges_Geschwindigkeit_Witter"]
    is_valid_by_weather_conditions = weather_condition == 0 or ( raining and weather_condition == 1 )

    checks = [
      is_valid_by_time,
      is_valid_by_day,
      is_valid_by_weather_conditions,
    ]
      
    return all(checks)

  def _dfs(self, seg_id, total_dist, visited, current_speed_ms, best_result):
    if seg_id in visited:
      return
    visited.add(seg_id)
  
    seg = self.predicative_segments.get(seg_id)
    if not seg:
      return
  
    speed_kmh = seg.get("Speed", NOT_SET)
    if seg.get("QualityFlag", False) and speed_kmh != NOT_SET:
      if speed_kmh < self.v_limit_output_last:
        v_target_ms = speed_kmh * CV.KPH_TO_MS
        braking_distance = (current_speed_ms**2 - v_target_ms**2) / (2 * DECELERATION_PREDICATIVE)
        
        if braking_distance > 0 and total_dist <= braking_distance:
          if speed_kmh < best_result["limit"]:
            best_result["limit"] = speed_kmh
            best_result["dist"] = total_dist

    children = [sid for sid, s in self.predicative_segments.items() if s.get("ID_Prev") == seg_id]
    if len(children) > 1:
      return  # Split detected, can not decide unique limit on current path
  
    for next_id in children:
      if seg_id == self.current_predicative_segment.get("ID"):
        next_length = self.current_predicative_segment.get("Length", 0)
      else:
        next_length = seg.get("Length", 0)
        
      self._dfs(next_id, total_dist + next_length, visited.copy(), current_speed_ms, best_result)

  def _get_speed_limit_psd_next(self, current_speed_ms):      
    current_id = self.current_predicative_segment.get("ID")
    self.v_limit_psd_next = NOT_SET

    if current_id == NOT_SET:
      return

    best_result = {"limit": float('inf'), "dist": float('inf')}
    self._dfs(current_id, 0, set(), current_speed_ms, best_result)

    now = time.time()
    if best_result["limit"] != float('inf'):
      self.v_limit_psd_next = best_result["limit"]
      self.v_limit_psd_next_last = best_result["limit"]
      self.v_limit_psd_next_last_timestamp = now
      self.v_limit_psd_next_decay_time = math.sqrt(2 * best_result["dist"] / DECELERATION_PREDICATIVE)
    else:
      if now - self.v_limit_psd_next_last_timestamp <= self.v_limit_psd_next_decay_time and self.v_limit_output_last > self.v_limit_psd_next_last and not self.v_limit_changed:
        self.v_limit_psd_next = self.v_limit_psd_next_last
      else:
        self.v_limit_psd_next_last = NOT_SET
        self.v_limit_psd_next_decay_time = NOT_SET

  def _get_speed_limit_psd(self):
    seg_id = self.current_predicative_segment.get("ID")
    if seg_id == NOT_SET:
      self.v_limit_psd = NOT_SET
      return

    seg = self.predicative_segments.get(seg_id)
    if seg and seg.get("Speed") != NOT_SET:
      self.v_limit_psd = seg.get("Speed")

  def _get_street_type(self, strassenkategorie, bebauung):
    street_type = NOT_SET

    if strassenkategorie == 1: # base type: urban
      street_type = STREET_TYPE_URBAN

    elif strassenkategorie in (2, 3, 4): # base type: non urban
      if bebauung == 1:
        street_type = STREET_TYPE_URBAN
      else:
        street_type = STREET_TYPE_NONURBAN

    elif strassenkategorie == 5: # base type: highway
      street_type = STREET_TYPE_HIGHWAY

    return street_type

  def _receive_speed_limit_psd_legal(self, psd_06):
    if psd_06["PSD_06_Mux"] == 2:
      if psd_06["PSD_Ges_Typ"] == 2:
        street_type = self.current_predicative_segment["StreetType"]
        if ((psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_URBAN    and street_type == STREET_TYPE_URBAN) or
            (psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_NONURBAN and street_type == STREET_TYPE_NONURBAN) or
            (psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_HIGHWAY  and street_type == STREET_TYPE_HIGHWAY)):
          raw_speed = psd_06["PSD_Ges_Geschwindigkeit"]
          self.v_limit_psd_legal = self._convert_raw_speed_psd(raw_speed, street_type)
