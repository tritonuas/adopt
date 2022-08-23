import omtools.api as ot
import numpy as np

class CalcWingArea(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    wingpsan = self.declare_input('wing_span', val=11)
    mac = self.declare_input('wing_mean_aerodynamic_chord', val=1)

    wing_area = wingpsan*mac
    wing_aspect_ratio = wingpsan**2/wing_area

    self.register_output('wing_area', wing_area)
    self.register_output('wing_aspect_ratio', wing_aspect_ratio)
