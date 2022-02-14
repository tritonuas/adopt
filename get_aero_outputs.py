import omtools.api as ot
import numpy as np

class GetAeroOutputs(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    air_density = self.declare_input('air_density', val=1.225)

    wing_area = self.declare_input('wing_area')

    k = self.declare_input('k', val=1/(np.pi*0.9*8))
    cL =  self.declare_input('cL', val=0.5)
    cD0 = self.declare_input('cD0', val=0.3)
    # cL_max =  self.declare_input('cL_max', val=1.77)
    sweep_angle = self.declare_input('sweep_angle', val=0.)

    lift = self.declare_input('lift')
    climb_gradient = self.declare_input('min_climb_gradient', val=0.1)
    cD0 = self.declare_input('cD0', val=0.02)
    e = self.declare_input('oswald_eff_factor', val=0.8)
    wing_ar = self.declare_input('wing_ar', val=11.)
    velocity_cruise = self.declare_input('velocity_cruise', val=50.)
    eta = self.declare_input('total_propulsive_efficiency', val=0.9685)
    n = self.declare_input('ultimate_load_factor', val=3.)
    # n = 3.
    cDi = k*cL**2
    cD = cD0 + cDi
    l_over_d = cL/cD

    cL_max = (cL+1.25)*0.9*ot.cos(sweep_angle)
    velocity_stall = (2*lift/(air_density*wing_area*cL_max))**(1/2)
    thrust_per_weight_req_climb = climb_gradient + 2*(cD0/(np.pi*e*wing_ar))**(1/2)
    power_per_weight_req_climb = thrust_per_weight_req_climb*velocity_cruise/eta
    thrust_req_climb = thrust_per_weight_req_climb*lift
    thrust_per_weight_req_maneuver = 2*n*(cD0/(np.pi*e*wing_ar))**(1/2)
    power_per_weight_req_maneuver = thrust_per_weight_req_maneuver*velocity_cruise/eta
    thrust_req_maneuver = thrust_per_weight_req_maneuver*lift

    wing_loading = lift/wing_area
    turning_radius = velocity_cruise**2/(9.81*(cL_max/cL**2-1)**(1/2))

    self.register_output('cDi', cDi)
    self.register_output('cD', cD)
    self.register_output('l_over_d', l_over_d)

    self.register_output('velocity_stall', velocity_stall)

    self.register_output('wing_loading', wing_loading)

    self.register_output('power_per_weight_req_climb', power_per_weight_req_climb)
    self.register_output('power_per_weight_req_maneuver', power_per_weight_req_maneuver)
    self.register_output('thrust_req_climb', thrust_req_climb)
    self.register_output('thrust_req_maneuver', thrust_req_maneuver)
    self.register_output('turning_radius', turning_radius)

    