import omtools.api as ot
import numpy as np

class FuseWeightAnalyticModel(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):

    carbon_epoxy_density = self.declare_input("carbon_epoxy_density")
    fuse_num_plies = self.declare_input("fuse_num_plies")
    bulkhead_num = self.declare_input("bulkhead_num")
    bulkhead_thickness = self.declare_input("bulkhead_thickness")
    bulkhead_area_fraction = self.declare_input("bulkhead_area_fraction")
    plywood_density = self.declare_input("plywood_density")
    fuse_height = self.declare_input("fuse_height")
    fuse_width = self.declare_input("fuse_width")
    fuse_length = self.declare_input("fuse_length")
    fuse_weight_fudge_factor = self.declare_input("fuse_weight_fudge_factor")


    carbon_epoxy_thickness = 0.0003
    bulkhead_area_density = plywood_density*bulkhead_num*bulkhead_area_fraction*bulkhead_thickness # kg/m^2   
    carbon_epoxy_area_density = carbon_epoxy_density*fuse_num_plies*carbon_epoxy_thickness
    
    fuse_weight = carbon_epoxy_area_density*(2*fuse_length*fuse_width+2*fuse_length*fuse_height+2*fuse_height*fuse_width) + fuse_height*fuse_width*bulkhead_area_density
    skin = carbon_epoxy_area_density*(2*fuse_length*fuse_width+2*fuse_length*fuse_height+2*fuse_height*fuse_width) # these are defined in matlab
    bulkhead = fuse_height*fuse_width*bulkhead_area_density                                                        # these are defined in matlab
    
    fuse_weight = fuse_weight*9.81 # conversion from kg to newtons
    fuse_weight = fuse_weight_fudge_factor*fuse_weight
    self.register_output("fuse_weight", fuse_weight)
