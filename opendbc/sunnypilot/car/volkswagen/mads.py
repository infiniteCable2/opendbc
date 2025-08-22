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

  def update_mads(self, ret: structs.CarState, can_parser_pt: CANParser) -> None:
    self.prev_lkas_button = self.lkas_button

    # block mads from detecting a cruise state transition in parked mode while cruise is temporary not available
    parking_brake_not_fully_open = can_parser_pt.vl["ESC_50"]["EPB_Status"] in (1, 3, 4)
    temp_cruise_fault = can_parser_pt.vl["Motor_51"]["TSK_Status"] == 6
    if temp_cruise_fault and parking_brake_not_fully_open:
      ret.cruiseState.available = True
    
    # some newer gen MEB cars do not have a main cruise button and a native cancel button is present
    # cruise state can not be fully toggled off for these cars!    
    user_disable = False
         
    for b in ret.buttonEvents:
      if b.type == ButtonType.cancel and b.pressed: # on rising edge
        user_disable = True
        break
    
    steering_enabled = can_parser_pt.vl["QFK_01"]["LatCon_HCA_Status"] == "ACTIVE" # presume mads is actively steering
    self.lkas_button = steering_enabled and user_disable