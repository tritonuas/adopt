import csdl
import numpy as np

class DragPolarModel(csdl.Model):
  
  def initialize(self):
    pass

  def define(self):
    air_density = self.declare_variable('air_density', val=1.225)

    wing_area = self.declare_variable('wing_area', val=11.)
    wing_aspect_ratio =  self.declare_variable('wing_aspect_ratio', val=11.)
    default_gross_mass = 13.5
    g = 9.81
    gross_weight = self.declare_variable('gross_weight', val=default_gross_mass*g)
    
    cD0 = self.declare_variable('cD0', val=0.35)
    e = self.declare_variable('oswald_eff_factor', val=0.730947)

    k = 1/(np.pi*e*wing_aspect_ratio)

    cL = (cD0/k)**(1/2)
  
    # L = gross_weight = 1/2*air_density*velocity_cruise**2*wing_area*cL
    velocity_cruise = (gross_weight/(1/2*air_density*wing_area*cL))**(1/2)
    lift = 1/2*air_density*velocity_cruise**2*wing_area*cL

    self.register_output('cL', cL)
    self.register_output('velocity_cruise', velocity_cruise)
    self.register_output('k', k)
    self.register_output('lift', lift)
