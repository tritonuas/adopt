import omtools.api as ot
import numpy as np

class WingWeightAnalyticModel(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):

    carbon_epoxy_thickness = self.declare_input("carbon_epoxy_thickness")
    carbon_epoxy_density = self.declare_input("carbon_epoxy_density")
    
    divinycell_density = self.declare_input("divinycell_density")
    divinycell_thickness = self.declare_input("divinycell_thickness")
    
    balsa_density = self.declare_input("balsa_density")
    
    wing_tip_thickness = self.declare_input("wing_tip_thickness")
    wing_root_thickness = self.declare_input("wing_root_thickness")
    wing_spar_width = self.declare_input("wing_spar_width")
    wing_spar_num = self.declare_input("wing_spar_num")
    wing_span = self.declare_input("wing_span")
    wing_area = self.declare_input("num_wing_spar")
    wing_num_plies = self.declare_input("wing_num_plies")
    wing_weight_fudge_factor = self.declare_input("wing_weight_fudge_factor")




    carbon_epoxy_thickness = 0.0003
    carbon_epoxy_area_density = carbon_epoxy_density*wing_num_plies*carbon_epoxy_thickness  # kg/m^2 
    divinycell_area_density = divinycell_thickness*divinycell_density                       # kg/m^2
    wing_spar_length_density = ((wing_tip_thickness+wing_root_thickness)/2)*wing_spar_num*balsa_density*wing_spar_width # kg/m
    
    
    wing_weight = 2*wing_area*(carbon_epoxy_area_density+divinycell_area_density) + wing_span*wing_spar_length_density # multiply area density by 2 to account for the top and bottom wing skins
    wing_weight = 9.81*wing_weight # convert from kg to N
    wing_weight = wing_weight*wing_weight_fudge_factor 

    self.register_output("wing_weight", wing_weight)
