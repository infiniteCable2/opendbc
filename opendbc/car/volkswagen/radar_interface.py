import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import DBC, VolkswagenFlags

# radar object drel is not end of radar facing side but probably the longitudinal center of object
# dbc drel offset of -3.6m is measured for a point mass type object (person)
# drel uncertainty statically subtracts something in beetwen the first half to end of a typical car length 
DREL_FRONT_EDGE_MARGIN = 1.5 # in m

RADAR_ADDR = 0x24F
NO_OBJECT = 0
DISTANCE_STATUS_VALID = 0
RADAR_UNAVAILABLE_THRESH = 5
LANE_TYPES = ("Same_Lane", "Left_Lane", "Right_Lane")
SIGNAL_SETS = tuple(
  (
    f"{prefix}_ObjectID",
    f"{prefix}_Long_Distance",
    f"{prefix}_Lat_Distance",
    f"{prefix}_Rel_Velo",
  )
  for lane in LANE_TYPES
  for idx in (1, 2)
  for prefix in (f"{lane}_0{idx}",)
)


def get_radar_can_parser(CP):
  if not (CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO)):
    return None

  if CP.flags & VolkswagenFlags.DISABLE_RADAR:
    return None

  if CP.flags & VolkswagenFlags.MQB_EVO_GEN2: # generation specific
    return None

  messages = [("Strukturen_01", 25)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 2)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    self.updated_messages: set[int] = set()
    self.trigger_msg: int = RADAR_ADDR
    self._track_id_counter: int = 0
    self.radar_unavailable_cnt: int = 0

    self.radar_off_can: bool = CP.radarUnavailable
    self.rcp: CANParser | None = get_radar_can_parser(CP)

    self._pts = self.pts

  def update(self, can_strings):
    """Entrypoint called by the vehicle loop every CAN tick."""
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    radar_data = self._process_radar_frame()
    self.updated_messages.clear()
    return radar_data

  def _process_radar_frame(self):
    ret = structs.RadarData()

    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret

    msg = self.rcp.vl["Strukturen_01"]
    get = msg.__getitem__

    # Radar reports its overall validity via Distance_Status (0 = Valid, 3 = Invalid).
    # Treat consecutive invalid reports as a temporary radar unavailability, similar to Ford MRR.
    if get("Distance_Status") != DISTANCE_STATUS_VALID:
      self.radar_unavailable_cnt += 1
    else:
      self.radar_unavailable_cnt = 0

    if self.radar_unavailable_cnt >= RADAR_UNAVAILABLE_THRESH:
      self._pts.clear()
      ret.errors.radarUnavailableTemporary = True
      return ret

    active_objects: dict[int, tuple[float, float, float]] = {}
    for obj_id_sig, long_sig, lat_sig, vel_sig in SIGNAL_SETS:
      obj_id = get(obj_id_sig)
      if obj_id == NO_OBJECT:
        continue

      if obj_id in active_objects:
        ret.errors.canError = True
        return ret

      active_objects[obj_id] = (
        get(long_sig),  # dRel
        get(lat_sig),   # yRel
        get(vel_sig),   # vRel
      )

    for obj_id, (d_rel, y_rel, v_rel) in active_objects.items():
      if obj_id not in self._pts:
        pt = structs.RadarData.RadarPoint()
        pt.trackId = self._track_id_counter
        self._track_id_counter += 1
        self._pts[obj_id] = pt
      else:
        pt = self._pts[obj_id]

      pt.dRel = d_rel - DREL_FRONT_EDGE_MARGIN
      pt.yRel = y_rel
      pt.vRel = v_rel

    inactive_ids = self._pts.keys() - active_objects.keys()
    for obj_id in inactive_ids:
      self._pts.pop(obj_id, None)

    ret.points = list(self._pts.values())
    return ret
