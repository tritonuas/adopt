import omtools.api as ot
import numpy as np

class VsWeightAnalyticModel(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):

    vs_spar_num= self.declare_input("vs_spar_num")
    vs_spar_width = self.declare_input("vs_spar_width")
    
    carbon_epoxy_density= self.declare_input("carbon_epoxy_density")
    blue_foam_density= self.declare_input("blue_foam_density")
    
    vs_root_thickness= self.declare_input("vs_root_thickness")
    vs_tip_thickness= self.declare_input("vs_tip_thickness")
    vs_area = self.declare_input("vs_area")
    vs_span = self.declare_input("vs_span")
    vs_num_plies = self.declare_input("vs_num_plies")
    vs_weight_fudge_factor = self.declare_input("vs_weight_fudge_factor")

    
    carbon_epoxy_thickness = 0.0003
    carbon_epoxy_area_density = carbon_epoxy_density*vs_num_plies*carbon_epoxy_thickness
    
    vs_spar_length_density = ((vs_tip_thickness+vs_root_thickness)/2)*vs_spar_num*blue_foam_density*vs_spar_width 
    
    vs_weight = 2*vs_area*carbon_epoxy_area_density + vs_span*vs_spar_length_density # multiply area density by 2 to account for the top and bottom tail skins
    skin = 2*vs_area*carbon_epoxy_area_density # these are defined in matlab
    vertical_stabilizer = vs_span*vs_spar_length_density     # these are defined in matlab
    
    vs_weight = 9.81*vs_weight # convert from adopt.kg to N
    vs_weight = vs_weight*vs_weight_fudge_factor 

    self.register_output("vs_weight", vs_weight)
