from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP, CP_IC):
    super().__init__(CP, CP_SP, CP_IC)

  def update(self, *_) -> tuple[structs.CarState, structs.CarStateSP, structs.CarStateIC]:
    return structs.CarState(), structs.CarStateSP(), structs.CarStateIC()
