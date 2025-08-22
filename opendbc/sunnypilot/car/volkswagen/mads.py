"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus,structs

from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser

ButtonType = structs.CarState.ButtonEvent.Type


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.cruise_initialized = False

  def update_mads(self, ret: structs.CarState, can_parser_pt: CANParser) -> None:
    self.prev_lkas_button = self.lkas_button
    user_enable = False
    user_disable = False
    
    # initially block temp fault when parked to prevent mads self activation when car removes the temp fault by switching into a drive mode
    if not self.cruise_initialized and ret.cruiseState.available or not ret.parkingBrake:
      self.cruise_initialized = True
      
    if not self.cruise_initialized:
      ret.cruiseState.available = True
      
    # some newer gen MEB cars do not have a main cruise button and a native cancel button is present      
    for b in ret.buttonEvents:
      if b.type == ButtonType.cancel and b.pressed: # on rising edge
        user_disable = True
      elif b.type in (ButtonType.setCruise, ButtonType.resumeCruise) and not b.pressed: # on falling edge
        user_enable = True
    
    steering_enabled = can_parser_pt.vl["QFK_01"]["LatCon_HCA_Status"] == "ACTIVE" # presume mads is actively steering
    
    lat_cancel_action = steering_enabled and user_disable
    lat_enable_action = not steering_enabled and user_enable
    
    self.lkas_button = True if lat_cancel_action or lat_enable_action else False # switch mads state