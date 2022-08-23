import omtools.api as ot
import numpy as np

class HsWeightAnalyticModel(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):

    blue_foam_density = self.declare_input("blue_foam_density")
    carbon_epoxy_density = self.declare_input("carbon_epoxy_density")
    hs_spar_num = self.declare_input("hs_spar_num")
    hs_spar_width= self.declare_input("hs_spar_width")
    hs_num_plies = self.declare_input("hs_num_plies")
    hs_area = self.declare_input("hs_area")
    hs_root_thickness = self.declare_input("hs_root_thickness")
    hs_tip_thickness= self.declare_input("hs_tip_thickness")
    hs_span = self.declare_input("hs_span")
    hs_weight_fudge_factor = self.declare_input("hs_weight_fudge_factor")


    carbon_epoxy_thickness = 0.0003
    carbon_epoxy_area_density = carbon_epoxy_density*hs_num_plies*carbon_epoxy_thickness
    
    hs_spar_length_density = ((hs_tip_thickness+hs_root_thickness)/2)*hs_spar_num*blue_foam_density*hs_spar_width
    
    hs_weight = 2*hs_area*carbon_epoxy_area_density + hs_span*hs_spar_length_density # multiply area density by 2 to account for the two sides of tail skins
    skin = 2*hs_area*carbon_epoxy_area_density # these are defined in matlab
    htail = hs_span*hs_spar_length_density     # these are defined in matlab
    
    hs_weight = 9.81*hs_weight # convert from adopt.kg to N
    hs_weight = hs_weight*hs_weight_fudge_factor

    self.register_output("hs_weight", hs_weight)
