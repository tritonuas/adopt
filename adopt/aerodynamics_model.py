import csdl
import numpy as np

from adopt.empirical_gross_weight_model import EmpiricalGrossWeightModel
from adopt.oswald_efficiency_factor_model import OswaldEfficiencyFactorModel
from adopt.velocity_coupling_model import VelocityCouplingModel

class AerodynamicsModel(csdl.Model):

  def initialize(self):
    pass
  
  def define(self):

    # Approximate oswald efficiency factor
    oswald_efficiency_factor_model = OswaldEfficiencyFactorModel()
    self.add(submodel=oswald_efficiency_factor_model, name='oswald_efficiency_factor_model')

    # Above two models in a model
    velocity_coupling_model = VelocityCouplingModel()
    self.add(submodel=velocity_coupling_model, name='velocity_coupling_model')

