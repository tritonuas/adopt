import openmdao.api as om
import omtools.api as ot
import numpy as np

class LamFunc(ot.Group):
  def initialize():
    pass

  def setup(self):
    density = self.declare_input('air_density', val=1.225)
    viscosity = self.declare_input('air_viscosity', val=1.789e-5)
    velocity = self.declare_input('velocity_cruise', val=50.)
    x = self.declare_input('x')

    cf_lam_comp = 1.328/(density*velocity*x/viscosity)^(1/2)

    self.register_output('cf_lam_comp', cf_lam_comp)