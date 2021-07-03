import omtools.api as ot
import numpy as np

class CalcRange(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    eta = self.declare_input('total_propulsive_efficiency', val=0.9685)
    g = self.declare_input('g', val=9.81)
    battery_energy_density = self.declare_input('battery_energy_density', val=500)
    l_over_d = self.declare_input('l_over_d', val=15)
    battery_weight = self.declare_input('battery_weight_cruise', val=400*9.81)
    gross_weight = self.declare_input('gross_weight', val=1400*9.81)

    range = eta/g*battery_energy_density*l_over_d*battery_weight/gross_weight

    self.register_output('range', range)
