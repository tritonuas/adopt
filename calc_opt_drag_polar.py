import omtools.api as ot
import numpy as np

class CalcOptDragPolar(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    air_density = self.declare_input('air_density', val=1.225)

    wing_area = self.declare_input('wing_area', val=11.)
    ar =  self.declare_input('ar', val=11.)
    gross_weight = self.declare_input('gross_weight', val=1400.)
    
    cD0 = self.declare_input('cD0', val=0.35)
    e = self.declare_input('oswald_eff_factor', val=0.730947)

    k = 1/(np.pi*e*ar)

    cL = (cD0/k)**(1/2)
  
    # L = gross_weight = 1/2*air_density*velocity_cruise**2*wing_area*cL
    velocity_cruise = (gross_weight/(1/2*air_density*wing_area*cL))**(1/2)
    lift = 1/2*air_density*velocity_cruise**2*wing_area*cL

    self.register_output('cL', cL)
    self.register_output('velocity_cruise', velocity_cruise)
    self.register_output('k', k)
    self.register_output('lift', lift)
