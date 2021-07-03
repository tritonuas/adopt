import omtools.api as ot
import openmdao.api as om
import numpy as np
import unitconv as unit

class CalcGrossWeight(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    payload_weight = self.declare_input('payload_weight', val=500*9.81)
    battery_weight_cruise = self.declare_input('battery_weight_cruise', val=700*9.81)
    battery_weight_vtol = self.declare_input('battery_weight_vtol', val=100*9.81)
    vtol_motor_weight = self.declare_input('vtol_motor_weight', val=10*25*9.81)
    cruise_motor_weight = self.declare_input('cruise_motor_weight', val=71*9.81)

    gross_weight = self.declare_input('gross_weight', val=1400*9.81)
    cruise_velocity = self.declare_input('velocity_cruise', val=50.)
    air_density = self.declare_input('air_density', val=1.225)

    q = 1/2*air_density*cruise_velocity**2
    fudge = 0.9
    misc_fudge = 1.05
    load_fact_ult = 4.

    S_wing = self.declare_input('wing_area', val=11.)
    W_bwing = self.declare_input('wing_battery_weight_ratio', val=1)
    W_ar = self.declare_input('ar', val=11.)
    W_sweep = self.declare_input('sweep_angle', val=0.)
    wing_taper = self.declare_input('wing_taper_ratio', val=0.45)
    wing_t_over_c = self.declare_input('wing_t_c', val=0.12)

    S_htail = self.declare_input('htail_area')
    htail_ar = self.declare_input('htail_ar')
    htail_sweep = self.declare_input('htail_sweep')
    htail_taper = self.declare_input('htail_taper')
    htail_t_over_c = self.declare_input('htail_t_c', val=0.12)

    S_vtail = self.declare_input('vtail_area')
    vtail_ar = self.declare_input('vtail_ar')
    vtail_sweep = self.declare_input('vtail_sweep')
    vtail_taper = self.declare_input('vtail_taper')
    vtail_t_over_c = self.declare_input('vtail_t_c', val=0.12)

    fus_wet = self.declare_input('fuse_wetted')
    wing_cg = self.declare_input('wing_cg')
    htail_cg = self.declare_input('htail_cg')
    fus_len = self.declare_input('fuse_length')
    fus_str_depth = self.declare_input('fuse_depth')

    tail_boom_len = self.declare_input('tail_boom_length')
    tail_boom_rad = self.declare_input('tail_boom_radius')
    tail_boom_thick = 0.0035
    carbon_density = 1610

    tail_len = (htail_cg - wing_cg) * fus_len

    W_wing = 0.036 * (S_wing*unit.mti_area)**0.758 * (W_bwing*battery_weight_cruise*unit.mti_weight)**0.0035 * \
            (W_ar/ot.cos(W_sweep)**2)**0.6 * (q*unit.mti_press)**0.006 * \
            wing_taper**0.04 * (100*wing_t_over_c/ot.cos(W_sweep))**(-0.3) * \
            (load_fact_ult*gross_weight*unit.mti_weight)**0.49 * 1/unit.mti_weight * fudge

    W_htail = 0.016 * (load_fact_ult*gross_weight*unit.mti_weight)**0.414 * \
            (q*unit.mti_press)**0.168 * (S_htail*unit.mti_area)**0.896 * \
            (100*htail_t_over_c/ot.cos(htail_sweep))**(-0.12) * \
            (htail_ar/ot.cos(htail_sweep)**2)**0.043 * htail_taper**(-0.02) * \
            1/unit.mti_weight * fudge

    W_vtail = 0.073 * (load_fact_ult*gross_weight*unit.mti_weight)**0.376 * \
            (q*unit.mti_press)**0.122 * (S_vtail*unit.mti_area)**0.873 * \
            (100*vtail_t_over_c/ot.cos(vtail_sweep))**(-0.49) * \
            (vtail_ar/ot.cos(vtail_sweep)**2)**0.357 * vtail_taper**0.039 * \
            1/unit.mti_weight * fudge

    W_fuse = 0.052 * (fus_wet*unit.mti_area)**1.086 * \
            (load_fact_ult*gross_weight*unit.mti_weight)**0.177 * \
            (tail_len*unit.mti_len)**(-0.051) * (fus_len/fus_str_depth)**(-0.072) * \
            (q*unit.mti_press)**0.241 * 1/unit.mti_weight * fudge

    tail_boom_fudge = 1.5
    W_tailb = 2 * tail_boom_fudge * 2 *np.pi*tail_boom_len*tail_boom_rad * \
            tail_boom_thick * carbon_density

    empty_weight = W_wing + W_htail + W_vtail + W_fuse + W_tailb
    gross_weight = misc_fudge * (payload_weight + empty_weight + \
            battery_weight_cruise + battery_weight_vtol + vtol_motor_weight + \
            cruise_motor_weight)
    misc_weight = gross_weight * (misc_fudge - 1) / misc_fudge
    motor_weight = vtol_motor_weight + cruise_motor_weight
    # num_motors = 12
    # motor_weight = 0.75 * misc_weight / num_motors
    # misc_weight = 0.25 * misc_weight

    empty_weight = gross_weight - payload_weight

    self.register_output('gross_weight', gross_weight)
    self.register_output('empty_weight', empty_weight)
    self.register_output('wing_weight', W_wing)
    self.register_output('htail_weight', W_htail)
    self.register_output('vtail_weight', W_vtail)
    self.register_output('fuse_weight', W_fuse)
    self.register_output('tail_boom_weight', W_tailb)
    self.register_output('motor_weight', motor_weight)
    self.register_output('misc_weight', misc_weight)

    self.nonlinear_solver = om.NonlinearBlockGS()
    self.nonlinear_solver.options['iprint'] = 0
    self.nonlinear_solver.options['maxiter'] = 20

