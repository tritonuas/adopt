import openmdao.api as om
import omtools.api as ot
import numpy as np

from lam_func import LamFunc
from turb_func import TurbFunc

class CalcCf(ot.Group):
  def initialize(self):
    pass

  def setup(self):
    density = self.declare_input('air_density', val=1.225)
    viscosity = self.declare_input('air_viscosity', val=1.789e-5)
    a = self.declare_input('a', val=343)

    length = self.declare_input('length', val=1.)

    velocity = self.declare_input('velocity_cruise', val=50.)

    k = 0.7e-5
    RE_CRIT_TURB = 38.21*(length/k)**1.053

    x_crit_turb = RE_CRIT_TURB*viscosity/(velocity*density)

    if length > x_crit_turb:
        x = length
        self.register_output('x', x)
        lam_comp_group = LamFunc()
        self.add_subsystem('lam_comp_group', lam_comp_group, promotes=['*'])
        cf_lam_comp = self.declare_input('cf_lam_comp')
        turb_comp_group = TurbFunc()
        self.add_subsystem('turb_comp_group', turb_comp_group, promotes=['*'])
        cf_turb_comp = self.declare_input('cf_turb_comp')

        cf = (cf_lam_comp*x_crit_turb + \
            cf_turb_comp*(length-x_crit_turb)) \
            /length
    else:
        x = length
        cf = LamFunc()

    x = length
    turb_comp_group = TurbFunc()
    self.add_subsystem('turb_comp_group', turb_comp_group, promotes=['*'])
    cf_turb_comp = self.declare_input('cf_turb_comp')
    cf = cf_turb_comp
    self.register_output('cf', cf)
