import csdl
import numpy as np
import unitconv as unit

class EmpiricalGrossWeightModel(csdl.Model):
  
  def initialize(self):
    pass

  def define(self):
    payload_weight = self.declare_variable('payload_weight', val=500*9.81)
    battery_weight_cruise = self.declare_variable('battery_weight_cruise', val=700*9.81)
    wing_weight = self.declare_variable('wing_weight', val=2850*9.81)
#     battery_weight_vtol = self.declare_variable('battery_weight_vtol', val=100*9.81)
#     vtol_motor_weight = self.declare_variable('vtol_motor_weight', val=10*25*9.81)
    cruise_motor_weight = self.declare_variable('cruise_motor_weight', val=71*9.81)

    gross_weight_guess = self.declare_variable('gross_weight_guess', val=1400*9.81)
    cruise_velocity = self.declare_variable('velocity_cruise', val=20.)
    air_density = self.declare_variable('air_density', val=1.225)

    q = 1/2*air_density*cruise_velocity**2
    fudge = 0.9
    misc_fudge = 1.05
    load_fact_ult = 4.

    wing_area = self.declare_variable('wing_area', val=11.)
    wing_battery_weight_ratio = self.declare_variable('wing_battery_weight_ratio', val=1.)
    wing_aspect_ratio = self.declare_variable('wing_aspect_ratio', val=11.)
    wing_sweep = self.declare_variable('wing_sweep_angle', val=0.)
    wing_taper = self.declare_variable('wing_taper_ratio', val=0.45)
    wing_thickness_to_chord_ratio = self.declare_variable('wing_thickness_to_chord_ratio', val=0.12)

    horizontal_stabilizer_area = self.declare_variable('horizontal_stabilizer_area')
    horizontal_stabilizer_aspect_ratio = self.declare_variable('horizontal_stabilizer_aspect_ratio')
    horizontal_stabilizer_sweep = self.declare_variable('horizontal_stabilizer_sweep')
    horizontal_stabilizer_taper = self.declare_variable('horizontal_stabilizer_taper')
    horizontal_stabilizer_thickness_to_chord_ratio = self.declare_variable('horizontal_stabilizer_thickness_to_chord_ratio', val=0.12)

    vertical_stabilizer_area = self.declare_variable('vertical_stabilizer_area')
    vertical_stabilizer_aspect_ratio = self.declare_variable('vertical_stabilizer_aspect_ratio')
    vertical_stabilizer_sweep = self.declare_variable('vertical_stabilizer_sweep')
    vertical_stabilizer_taper = self.declare_variable('vertical_stabilizer_taper')
    vertical_stabilizer_thickness_to_chord_ratio = self.declare_variable('vertical_stabilizer_thickness_to_chord_ratio', val=0.12)

    fuselage_wetted = self.declare_variable('fuselage_wetted')
    wing_center_of_mass = self.declare_variable('wing_center_of_mass')
    horizontal_stabilizer_center_of_mass = self.declare_variable('horizontal_stabilizer_center_of_mass')
    fuselage_length = self.declare_variable('fuselage_length')
    fuselage_depth = self.declare_variable('fuselage_depth')

    tail_boom_length = self.declare_variable('tail_boom_length')
    tail_boom_radius = self.declare_variable('tail_boom_radius')
    tail_boom_thickness = 0.0035
    carbon_density = 1610

    tail_len = (horizontal_stabilizer_center_of_mass - wing_center_of_mass) * fuselage_length

#     wing_weight = 0.036 * (wing_area*unit.mti_area)**0.758 * (wing_battery_weight_ratio*battery_weight_cruise*unit.mti_weight)**0.0035 * \
#             (wing_aspect_ratio/csdl.cos(wing_sweep)**2)**0.6 * (q*unit.mti_press)**0.006 * \
#             wing_taper**0.04 * (100*wing_thickness_to_chord_ratio/csdl.cos(wing_sweep))**(-0.3) * \
#             (load_fact_ult*gross_weight*unit.mti_weight)**0.49 * 1/unit.mti_weight * fudge

#     wing_weight = (1410.2 + 1444.4)*9.81 wing weight is an independent component to get model working 

    horitontal_stabilizer_weight = 0.016 * (load_fact_ult*gross_weight_guess*unit.mti_weight)**0.414 * \
            (q*unit.mti_press)**0.168 * (horizontal_stabilizer_area*unit.mti_area)**0.896 * \
            (100*horizontal_stabilizer_thickness_to_chord_ratio/csdl.cos(horizontal_stabilizer_sweep))**(-0.12) * \
            (horizontal_stabilizer_aspect_ratio/csdl.cos(horizontal_stabilizer_sweep)**2)**0.043 * horizontal_stabilizer_taper**(-0.02) * \
            1/unit.mti_weight * fudge

    vertical_stabilizer_weight = 0.073 * (load_fact_ult*gross_weight_guess*unit.mti_weight)**0.376 * \
            (q*unit.mti_press)**0.122 * (vertical_stabilizer_area*unit.mti_area)**0.873 * \
            (100*vertical_stabilizer_thickness_to_chord_ratio/csdl.cos(vertical_stabilizer_sweep))**(-0.49) * \
            (vertical_stabilizer_aspect_ratio/csdl.cos(vertical_stabilizer_sweep)**2)**0.357 * vertical_stabilizer_taper**0.039 * \
            1/unit.mti_weight * fudge

    fuselage_weight = 0.052 * (fuselage_wetted*unit.mti_area)**1.086 * \
            (load_fact_ult*gross_weight_guess*unit.mti_weight)**0.177 * \
            (tail_len*unit.mti_len)**(-0.051) * (fuselage_length/fuselage_depth)**(-0.072) * \
            (q*unit.mti_press)**0.241 * 1/unit.mti_weight * fudge

    tail_boom_fudge = 1.5
    tail_boom_weight = 2 * tail_boom_fudge * 2 *np.pi*tail_boom_length*tail_boom_radius * \
            tail_boom_thickness * carbon_density

    empty_weight = wing_weight + horitontal_stabilizer_weight + vertical_stabilizer_weight + fuselage_weight + tail_boom_weight
    gross_weight = misc_fudge * (payload_weight + empty_weight + \
            battery_weight_cruise + cruise_motor_weight)
    misc_weight = gross_weight * (misc_fudge - 1) / misc_fudge
    motor_weight = cruise_motor_weight
    # num_motors = 12
    # motor_weight = 0.75 * misc_weight / num_motors
    # misc_weight = 0.25 * misc_weight

    empty_weight = gross_weight - payload_weight

    self.register_output('gross_weight', gross_weight)
    self.register_output('empty_weight', empty_weight)
#     self.register_output('wing_weight', wing_weight)
    self.register_output('horizontal_stabilizer_weight', horitontal_stabilizer_weight)
    self.register_output('vertical_stabilizer_weight', vertical_stabilizer_weight)
    self.register_output('fuselage_weight', fuselage_weight)
    self.register_output('tail_boom_weight', tail_boom_weight)
#     self.register_output('motor_weight', motor_weight)
    self.register_output('misc_weight', misc_weight)

#     self.nonlinear_solver = om.NonlinearBlockGS()
#     self.nonlinear_solver.options['iprint'] = 0
#     self.nonlinear_solver.options['maxiter'] = 20

