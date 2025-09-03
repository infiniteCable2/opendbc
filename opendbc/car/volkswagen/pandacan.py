def create_panda_data(packer, bus, roll):
  values = {
    "Roll": abs(roll),
    "Roll_VZ": 1 if roll > 0 else 0,
  }
  return packer.make_can_msg("Panda_Data_01", bus, values)
