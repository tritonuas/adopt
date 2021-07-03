import omtools.api as ot
import numpy as np

class CalcWingArea(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    wingpsan = self.declare_input('wingspan', val=11)
    mac = self.declare_input('wing_mac', val=1)

    wing_area = wingpsan*mac
    ar = wingpsan**2/wing_area

    self.register_output('wing_area', wing_area)
    self.register_output('ar', ar)
