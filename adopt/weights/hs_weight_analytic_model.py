import omtools.api as ot
import numpy as np

class HsWeightAnalyticModel(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):

    blue_foam_density = self.declare_input("blue_foam_density")
    carbon_epoxy_density = self.declare_input("carbon_epoxy_density")
    horizontal_stabilizer_spar_num = self.declare_input("horizontal_stabilizer_spar_num")
    horizontal_stabilizer_spar_width= self.declare_input("horizontal_stabilizer_spar_width")
    horizontal_stabilizer_num_plies = self.declare_input("horizontal_stabilizer_num_plies")
    horizontal_stabilizer_area = self.declare_input("horizontal_stabilizer_area")
    horizontal_stabilizer_root_thickness = self.declare_input("horizontal_stabilizer_root_thickness")
    horizontal_stabilizer_tip_thickness= self.declare_input("horizontal_stabilizer_tip_thickness")
    horizontal_stabilizer_span = self.declare_input("horizontal_stabilizer_span")
    horizontal_stabilizer_weight_fudge_factor = self.declare_input("horizontal_stabilizer_weight_fudge_factor")


    carbon_epoxy_thickness = 0.0003
    carbon_epoxy_area_density = carbon_epoxy_density*horizontal_stabilizer_num_plies*carbon_epoxy_thickness
    
    horizontal_stabilizer_spar_length_density = ((horizontal_stabilizer_tip_thickness+horizontal_stabilizer_root_thickness)/2)*horizontal_stabilizer_spar_num*blue_foam_density*horizontal_stabilizer_spar_width
    
    horizontal_stabilizer_weight = 2*horizontal_stabilizer_area*carbon_epoxy_area_density + horizontal_stabilizer_span*horizontal_stabilizer_spar_length_density # multiply area density by 2 to account for the two sides of tail skins
    skin = 2*horizontal_stabilizer_area*carbon_epoxy_area_density # these are defined in matlab
    htail = horizontal_stabilizer_span*horizontal_stabilizer_spar_length_density     # these are defined in matlab
    
    horizontal_stabilizer_weight = 9.81*horizontal_stabilizer_weight # convert from adopt.kg to N
    horizontal_stabilizer_weight = horizontal_stabilizer_weight*horizontal_stabilizer_weight_fudge_factor

    self.register_output("horizontal_stabilizer_weight", horizontal_stabilizer_weight)
