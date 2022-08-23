# import openmdao.api as om
# import omtools.api as ot
import csdl
import numpy as np

from adopt.lam_func import LamFunc
from adopt.turb_func import TurbFunc

class CalcCf(csdl.Model):
  def initialize(self):
    pass

  def define(self):
    # density = self.declare_input('air_density', val=1.225)
    # NOTE: Use create_input to create an independent variable that won't change in the optimization
    air_density = self.declare_variable('air_density', val=1.225)
    # viscosity = self.declare_input('air_viscosity', val=1.789e-5)
    air_viscosity = self.declare_variable('air_viscosity', val=1.789e-5)

    # length = self.declare_input('length', val=1.)
    # NOTE: Use declare_variable to create an input that gets fed in from adopt.another model
    length = self.declare_variable('length', val=1.)

    # velocity = self.declare_input('velocity_cruise', val=50.)
    velocity = self.declare_variable('velocity_cruise', val=20.)

    k = 0.7e-5
    RE_CRIT_TURB = 38.21*(length/k)**1.053

    x_crit_turb = RE_CRIT_TURB*air_viscosity/(velocity*air_density)

    if length > x_crit_turb:
        x = length
        self.register_output('x', x)
        laminar_flow_model = LamFunc()
        # self.add_subsystem('lam_comp_group', lam_comp_group, promotes=['*'])
        # NOTE: use add() to add a submodel to this model. 
        self.add('LaminarFlowModel', laminar_flow_model, promotes=['*'])
        # cf_lam_comp = self.declare_input('cf_lam_comp')
        cf_laminar_component = self.declare_input('cf_laminar_component')
        turbulent_flow_model = TurbFunc()
        # self.add_subsystem('TurbulentFlowModel', turb_comp_group, promotes=['*'])
        self.add('TurbulentFlowModel', turbulent_flow_model, promotes=['*'])
        # cf_turb_comp = self.declare_input('cf_turb_comp')
        cf_turbulent_component = self.declare_input('cf_turbulent_component')

        cf = (cf_laminar_component*x_crit_turb + \
            cf_turbulent_component*(length-x_crit_turb)) \
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
