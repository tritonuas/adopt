import omtools.api as ot
import numpy as np
import unitconv as unit

class CalcDynStab(ot.Group):

    def initialize(self):
        pass

    def setup(self):
        Rx = 0.25
        Ry = 0.38
        Rz = 0.39

        g = 9.81

        C_lq = 0.01    # This is a guess. I have no equation
        C_yr = 0.25     # Also a guess lol

        gross = self.declare_input('gross_weight', val=12*g)
        wingspan = self.declare_input('wingspan')
        fuse_len = self.declare_input('fuse_length', val = 1.1)

        Ixx = (wingspan*unit.mti_len)**2 * (gross*unit.mti_weight) * Rx**2 / \
                unit.g / 4 / unit.mti_inertia
        Iyy = (fuse_len*unit.mti_len)**2 * (gross*unit.mti_weight) * Ry**2 / \
                unit.g / 4 / unit.mti_inertia
        Izz = ((fuse_len+wingspan)/2*unit.mti_len)**2 * (gross*unit.mti_weight) * \
                Rz**2 / unit.g / 4 / unit.mti_inertia

        V_0 = self.declare_input('velocity_cruise', val=20.)
        air_density = self.declare_input('air_density', val=1.225)
        q = 1/2*air_density*V_0**2

        S_wing = self.declare_input('wing_area', val=1.)
        W_mac = self.declare_input('W_mac')
        wingspan = self.declare_input('wingpsan')

        C_Lalpha = self.declare_input('C_Lalpha')
        C_mq = self.declare_input('C_mq')
        C_malpha = self.declare_input('C_malpha')
        C_lbeta = self.declare_input('C_lbeta')
        C_lp = self.declare_input('C_lp')
        C_nbeta = self.declare_input('C_nbeta')
        C_ybeta = self.declare_input('C_ybeta')
        C_nr = self.declare_input('C_nr')

        m = gross/g
        Z_w = -q*S_wing/m*C_Lalpha/V_0
        M_q = q*W_mac**2*S_wing/2/V_0/Iyy*C_mq
        M_w = q*W_mac*S_wing/Iyy*C_malpha/V_0
        Z_q = -q*W_mac*S_wing/2/m/V_0*C_lq
        Y_v = q*S_wing/m*C_ybeta/V_0
        N_r = q*S_wing*wingspan**2/2/Izz/V_0*C_nr
        N_v = q*S_wing*wingspan/Izz*C_nbeta/V_0
        Y_r = q*S_wing*wingspan/2/m/V_0*C_yr
        L_p = q*S_wing*wingspan**2/2/Ixx/V_0*C_lp

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
