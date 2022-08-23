import csdl
import numpy as np

class LamFunc(csdl.Model):
  def initialize():
    pass

  def define(self):
    density = self.declare_variable('air_density', val=1.225)
    viscosity = self.declare_variable('air_viscosity', val=1.789e-5)
    velocity = self.declare_variable('velocity_cruise', val=20.)
    x = self.declare_variable('x')

    cf_lam_comp = 1.328/(density*velocity*x/viscosity)^(1/2)

    self.register_output('cf_lam_comp', cf_lam_comp)