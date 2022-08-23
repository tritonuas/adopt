import csdl
import numpy as np

class RangeModel(csdl.Model):
  
  def initialize(self):
    pass

  def define(self):
    eta = self.declare_variable('total_propulsive_efficiency', val=0.9685)
    g = self.declare_variable('g', val=9.81)
    battery_energy_density = self.declare_variable('battery_energy_density', val=(5200*14.8/1000*3600)/0.4365)
    l_over_d = self.declare_variable('l_over_d', val=15)
    battery_weight = self.declare_variable('battery_weight_cruise', val=436.5*4/1000*9.81)
    default_gross_mass = 13.5
    g = 9.81
    gross_weight = self.declare_variable('gross_weight', val=default_gross_mass*g)

    range = eta/g*battery_energy_density*l_over_d*battery_weight/gross_weight

    self.register_output('range', range)
