import csdl
import numpy as np
import unitconv as unit

class DynamicStabilityModel(csdl.Model):

    def initialize(self):
        pass

    def define(self):
        Rx = 0.25
        Ry = 0.38
        Rz = 0.39

        g = 9.81

        C_lq = 0.01    # This is a guess. I have no equation
        C_yr = 0.25     # Also a guess lol

        default_gross_mass = 13.5
        g = 9.81
        gross_weight = self.declare_variable('gross_weight', val=default_gross_mass*g)
        wing_span = self.declare_variable('wing_span')
        fuselage_len = self.declare_variable('fuselage_length', val = 1.1)

        Ixx = (wing_span*unit.mti_len)**2 * (gross_weight*unit.mti_weight) * Rx**2 / \
                unit.g / 4 / unit.mti_inertia
        Iyy = (fuselage_len*unit.mti_len)**2 * (gross_weight*unit.mti_weight) * Ry**2 / \
                unit.g / 4 / unit.mti_inertia
        Izz = ((fuselage_len+wing_span)/2*unit.mti_len)**2 * (gross_weight*unit.mti_weight) * \
                Rz**2 / unit.g / 4 / unit.mti_inertia

        V_0 = self.declare_variable('velocity_cruise', val=20.)
        air_density = self.declare_variable('air_density', val=1.225)
        q = 1/2*air_density*V_0**2

        wing_area = self.declare_variable('wing_area', val=1.)
        W_mean_aerodynamic_chord = self.declare_variable('W_mean_aerodynamic_chord')
        wing_span = self.declare_variable('wingpsan')

        C_Lalpha = self.declare_variable('C_Lalpha')
        C_mq = self.declare_variable('C_mq')
        C_malpha = self.declare_variable('C_malpha')
        C_lbeta = self.declare_variable('C_lbeta')
        C_lp = self.declare_variable('C_lp')
        C_nbeta = self.declare_variable('C_nbeta')
        C_ybeta = self.declare_variable('C_ybeta')
        C_nr = self.declare_variable('C_nr')

        m = gross_weight/g
        Z_w = -q*wing_area/m*C_Lalpha/V_0
        M_q = q*W_mean_aerodynamic_chord**2*wing_area/2/V_0/Iyy*C_mq
        M_w = q*W_mean_aerodynamic_chord*wing_area/Iyy*C_malpha/V_0
        Z_q = -q*W_mean_aerodynamic_chord*wing_area/2/m/V_0*C_lq
        Y_v = q*wing_area/m*C_ybeta/V_0
        N_r = q*wing_area*wing_span**2/2/Izz/V_0*C_nr
        N_v = q*wing_area*wing_span/Izz*C_nbeta/V_0
        Y_r = q*wing_area*wing_span/2/m/V_0*C_yr
        L_p = q*wing_area*wing_span**2/2/Ixx/V_0*C_lp

        omega_sp_2 = Z_w*M_q - M_w*(V_0+Z_q)
        zeta_sp_2 = (Z_w+M_q)**2/4/omega_sp_2
        omega_dr_2 = Y_v*N_r + N_v*(V_0-Y_r)
        zeta_dr_2 = (Y_v+N_r)**2/4/omega_dr_2
        tau_r = -1/L_p

        self.register_output('w_sp', omega_sp_2)
        self.register_output('z_sp', zeta_sp_2)
        self.register_output('w_dr', omega_dr_2)
        self.register_output('z_dr', zeta_dr_2)
        self.register_output('wd_dr', omega_dr_2*zeta_dr_2)
        self.register_output('t_roll', tau_r)
