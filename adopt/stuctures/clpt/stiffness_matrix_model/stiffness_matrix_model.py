import csdl
import numpy as np

class StiffnessMatrixModel(csdl.Model):
  
  def initialize(self):
    self.parameters.declare('num_plies')
    self.parameters.declare('radians_or_degrees', val='degrees')

  def define(self):
    num_plies = self.parameters['num_plies']
    radians_or_degrees = self.parameters['radians_or_degrees']

    ply_angles = self.declare_variable('ply_angles', val=np.zeros((num_plies,)))
    ply_thicknesses = self.declare_variable('ply_thicknesses', val=np.ones((num_plies,))*2.e-4)
    material_properties = self.declare_variable('material_properties', shape=(4,))

    # ABD Model
    # -- Construct Q_bar matrix (local stiffness matrix for each ply)
    for i in range(num_plies):
      ply_compliance_matrices[:,:,i] = get_s_planar_ortho(material_properties[:,i])
      ply_stiffness_matrices[:,:,i] = np.linalg.inv(S[:,:,i])
      ply_stiffness_matrices_global_frame[:,:,i] = get_Q_bar_planar(Q[:,:,i], ply_angles[i], radians_or_degrees)

    # -- Convert Q_bar to ABD (global compliance matrix)
    laminate_compliance_matrix = qBar_to_abd(qBar, ply_thicknesses)
    laminate_stiffness_matrix = np.linalg.inv(laminate_compliance_matrix)

    self.register_output('stiffness_matrix', laminate_stiffness_matrix)

    # Outputs: global_stiffness_matrix
