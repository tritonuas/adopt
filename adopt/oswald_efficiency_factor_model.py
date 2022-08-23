import csdl
import numpy as np


def emp_func(taper_ratio):
  return 0.0524*taper_ratio**4 - 0.15*taper_ratio**3 \
      + 0.1659*taper_ratio**2 - 0.0706*taper_ratio + 0.0119

def get_delta_lambda(wing_sweep_angle):
  return -0.357 + 0.45*csdl.exp(0.0375*wing_sweep_angle)

class OswaldEfficiencyFactorModel(csdl.Model):
  
  def initialize(self):
    pass

  def define(self):
    wing_aspect_ratio = self.declare_variable('wing_aspect_ratio', val=11)
    taper_ratio = self.declare_variable('taper_ratio', val=0.45)
    wing_sweep_angle = self.declare_variable('wing_sweep_angle', val=0.)

    # Raymer method
    # e = 1.78*(1 - 0.045*wing_aspect_ratio**0.68) - 0.64

    # Method described my researcher at Hamburg University
    delta_lambda = get_delta_lambda(wing_sweep_angle)
    e_theoretical = 1/(1 + emp_func(taper_ratio - delta_lambda)*wing_aspect_ratio)
    k_f = 0.971
    k_d0 = 0.804
    k_m = 1
    e = e_theoretical*k_f*k_d0*k_m

    self.register_output('oswald_efficiency_factor', e)
    