import omtools.api as ot
import numpy as np

class VsWeightAnalyticModel(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):

    vertical_stabilizer_spar_num= self.declare_input("vertical_stabilizer_spar_num")
    vertical_stabilizer_spar_width = self.declare_input("vertical_stabilizer_spar_width")
    
    carbon_epoxy_density= self.declare_input("carbon_epoxy_density")
    blue_foam_density= self.declare_input("blue_foam_density")
    
    vertical_stabilizer_root_thickness= self.declare_input("vertical_stabilizer_root_thickness")
    vertical_stabilizer_tip_thickness= self.declare_input("vertical_stabilizer_tip_thickness")
    vertical_stabilizer_area = self.declare_input("vertical_stabilizer_area")
    vertical_stabilizer_span = self.declare_input("vertical_stabilizer_span")
    vertical_stabilizer_num_plies = self.declare_input("vertical_stabilizer_num_plies")
    vertical_stabilizer_weight_fudge_factor = self.declare_input("vertical_stabilizer_weight_fudge_factor")

    
    carbon_epoxy_thickness = 0.0003
    carbon_epoxy_area_density = carbon_epoxy_density*vertical_stabilizer_num_plies*carbon_epoxy_thickness
    
    vertical_stabilizer_spar_length_density = ((vertical_stabilizer_tip_thickness+vertical_stabilizer_root_thickness)/2)*vertical_stabilizer_spar_num*blue_foam_density*vertical_stabilizer_spar_width 
    
    vertical_stabilizer_weight = 2*vertical_stabilizer_area*carbon_epoxy_area_density + vertical_stabilizer_span*vertical_stabilizer_spar_length_density # multiply area density by 2 to account for the top and bottom tail skins
    skin = 2*vertical_stabilizer_area*carbon_epoxy_area_density # these are defined in matlab
    vertical_stabilizer = vertical_stabilizer_span*vertical_stabilizer_spar_length_density     # these are defined in matlab
    
    vertical_stabilizer_weight = 9.81*vertical_stabilizer_weight # convert from adopt.kg to N
    vertical_stabilizer_weight = vertical_stabilizer_weight*vertical_stabilizer_weight_fudge_factor 

    self.register_output("vertical_stabilizer_weight", vertical_stabilizer_weight)
