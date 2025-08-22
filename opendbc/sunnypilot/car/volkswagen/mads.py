"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs

from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser
from opendbc.car.volkswagen.values import GearShifter

ButtonType = structs.CarState.ButtonEvent.Type

TOLERANCE_MAX     = 100 # 1 second
TEMP_CRUISE_FAULT = 6


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.tolerance_counter = TOLERANCE_MAX

  def update_mads(self, ret: structs.CarState, can_parser_pt: CANParser, hca_status) -> None:
    self.prev_lkas_button = self.lkas_button

    # safely block MADS from detecting a cruise state transition from parked mode while cruise is temporary not available
    # (blocking user unintended MADS enablement via main cruise state mechanism)
    temp_cruise_fault = can_parser_pt.vl["Motor_51"]["TSK_Status"] == TEMP_CRUISE_FAULT
    drive_mode        = ret.gearShifter == GearShifter.drive
    
    if temp_cruise_fault and ret.parkingBrake and not drive_mode:
      ret.cruiseState.available = True
      self.tolerance_counter    = 0
      
    elif self.tolerance_counter < TOLERANCE_MAX: # grant a little bit of time for the car doing its state transition
      ret.cruiseState.available = True
      self.tolerance_counter    = min(self.tolerance_counter + 1, TOLERANCE_MAX)
    
    # some newer gen MEB cars do not have a main cruise button and a native cancel button is present
    # WARNING: cruise state can not be fully toggled off for these cars!    
    user_disable = False
         
    for b in ret.buttonEvents:
      if b.type == ButtonType.cancel and b.pressed: # on rising edge
        user_disable = True
        break
    
    steering_enabled = hca_status == "ACTIVE" # assume mads is actively steering
    
    self.lkas_button = steering_enabled and user_disable