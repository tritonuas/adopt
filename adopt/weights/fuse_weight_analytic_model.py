import omtools.api as ot
import numpy as np

class FuseWeightAnalyticModel(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):

    carbon_epoxy_density = self.declare_input("carbon_epoxy_density")
    fuselage_num_plies = self.declare_input("fuselage_num_plies")
    bulkhead_num = self.declare_input("bulkhead_num")
    bulkhead_thickness = self.declare_input("bulkhead_thickness")
    bulkhead_area_fraction = self.declare_input("bulkhead_area_fraction")
    plywood_density = self.declare_input("plywood_density")
    fuselage_height = self.declare_input("fuselage_height")
    fuselage_width = self.declare_input("fuselage_width")
    fuselage_length = self.declare_input("fuselage_length")
    fuselage_weight_fudge_factor = self.declare_input("fuselage_weight_fudge_factor")


    carbon_epoxy_thickness = 0.0003
    bulkhead_area_density = plywood_density*bulkhead_num * bulkhead_area_fraction * bulkhead_thickness # kg/m^2   
    carbon_epoxy_area_density = carbon_epoxy_density * fuselage_num_plies * carbon_epoxy_thickness
    
    fuselage_weight = carbon_epoxy_area_density * (2 * fuselage_length * fuselage_width + 2 * fuselage_length * fuselage_height + 2 * fuselage_height*fuselage_width) + fuselage_height*fuselage_width * bulkhead_area_density
    skin = carbon_epoxy_area_density * (2 * fuselage_length * fuselage_width + 2 * fuselage_length * fuselage_height + 2 * fuselage_height * fuselage_width) # these are defined in matlab
    bulkhead = fuselage_height * fuselage_width * bulkhead_area_density                                                        # these are defined in matlab
    
    fuselage_weight = fuselage_weight * 9.81 # conversion from adopt.kg to newtons
    fuselage_weight = fuselage_weight_fudge_factor * fuselage_weight
    self.register_output("fuselage_weight", fuselage_weight)
