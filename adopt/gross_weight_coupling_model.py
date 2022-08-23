import csdl
import numpy as np

from adopt.empirical_gross_weight_model import EmpiricalGrossWeightModel
from adopt.oswald_efficiency_factor_model import OswaldEfficiencyFactorModel
from adopt.velocity_coupling_model import VelocityCouplingModel

class GrossWeightCouplingModel(csdl.Model):

  def initialize(self):
    pass
  
  def define(self):

    # empirical_gross_weight_model = EmpiricalGrossWeightModel()
    # self.add(submodel=empirical_gross_weight_model, name='empirical_gross_weight_model')

    # Approximate oswald efficiency factor
    oswald_efficiency_factor_model = OswaldEfficiencyFactorModel()
    self.add(submodel=oswald_efficiency_factor_model, name='oswald_efficiency_factor_model')

    # Above two models in a model
    velocity_coupling_model = VelocityCouplingModel()
    self.add(submodel=velocity_coupling_model, name='velocity_coupling_model')

    # # self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)
    # self.nonlinear_solver = om.NonlinearBlockGS()
    # # self.nonlinear_solver = om.NonlinearBlockJac()
    # self.nonlinear_solver.options['iprint'] = 0
    # self.nonlinear_solver.options['maxiter'] = 20
    # self.linear_solver = om.DirectSolver()
