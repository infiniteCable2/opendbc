from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP, CP_IC):
    super().__init__(dbc_names, CP, CP_SP, CP_IC)

  def update(self, CC, CC_SP, CC_IC, CS, now_nanos):
    return CC.actuators.as_builder(), []
