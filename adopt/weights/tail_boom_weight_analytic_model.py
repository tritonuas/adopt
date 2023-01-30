import numpy as np
import csdl

class TailBoomWeightAnalyticModel(csdl.Model):
  
  def initialize(self):
    pass

  def define(self):

    tail_boom_num_plies = self.declare_variable("tail_boom_num_plies")
    carbon_epoxy_density= self.declare_variable("carbon_epoxy_density")
    tail_boom_length = self.declare_variable("tail_boom_length")
    tail_boom_weight_fudge_factor = self.declare_variable("tail_boom_weight_fudge_factor")


    
    carbon_epoxy_thickness = 0.0003
    tailboom_radius = 0.0762/2
    carbon_epoxy_area_density = carbon_epoxy_density*tail_boom_num_plies*carbon_epoxy_thickness # kg/m^2 
    
    surface_area = tail_boom_length*2*np.pi*tailboom_radius
    tail_boom_weight = surface_area*carbon_epoxy_area_density
    tail_boom_weight = 9.81*tail_boom_weight # convert from adopt.kg to N
    tail_boom_weight = tail_boom_weight*tail_boom_weight_fudge_factor

    self.register_output("tail_boom_weight", tail_boom_weight)
