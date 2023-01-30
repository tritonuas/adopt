import csdl
import numpy as np

from adopt.stuctures.clpt.stiffness_matrix_model.stiffness_matrix_model import StiffnessMatrixModel

class CLPTModel(csdl.Model):
  
  def initialize(self):
    self.parameters.declare('num_plies')
    self.parameters.declare('radians_or_degrees', val='degrees')
    self.parameters.declare('failure_criterion', val='max_stress')  # This will be a modified max_stress that gives a differentiable output.
    self.parameters.declare('safety_factor', val=1.5)   # We should definitely use higher than 1.5
    self.parameters.declare('print_output', val=False)
    # Perhaps want to add a plotting boolean as well

  def define(self):
    num_plies = self.parameters['num_plies']
    radians_or_degrees = self.parameters['radians_or_degrees']
    failure_criterion = self.parameters['failure_criterion']
    safety_factor = self.parameters['safety_factor']
    print_output = self.parameters['print_output']


    ply_angles = self.declare_variable('ply_angles', val=np.zeros((num_plies,)))
    ply_thicknesses = self.declare_variable('ply_thicknesses', val=np.ones((num_plies,))*2.e-4)
    material_properties = self.declare_variable('material_properties', shape=(4,))

    # CLPT Model
    # -- expand material properties to all plies
    material_properties = csdl.expand(material_properties, shape=tuple(num_plies,) + material_properties.shape, indices='i->ji')

    # -- get thermal load (cut out, won't add)

    # -- calculate ABD matrix
    stiffness_matrix_model = StiffnessMatrixModel(num_plies=num_plies, radians_or_degrees=radians_or_degrees)
    self.add(submodel=stiffness_matrix_model, name='stiffness_matrix_model')
    
    # -- calculate strains and curvatures for the middle of each ply
    # -- calculate global stresses at top and bottom of each ply using the strains and curvatures
    # -- calculate thermal strain (cut out since no thermal load)
    # -- get mechanical strains by subtracting out thermal strain (cut out since now it doesn't do anything)
    # -- for each ply, convert global stresses to local stresses because local determines failure


    # Report ply failures (if print_output is true)
    if print_output:
        pass

    # Outputs: MS, failed_plies, failed_sides, failed_Zs, failure_modes, failure_tension_compression_or_shear
