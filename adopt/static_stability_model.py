import csdl
import numpy as np

class StaticStabilityModel(csdl.Model):

    def initialize(self):
        pass

    def define(self):
        payload_weight = self.declare_variable('payload_weight', val=400*9.81)
        battery_weight = self.declare_variable('battery_weight_cruise', val=300*9.81)
        # battery_weight_vtol = self.declare_variable('battery_weight_vtol', val=500*9.81)
        # vtol_motor_weight = self.declare_variable('vtol_motor_weight', val=25*10*9.81)
        cruise_motor_weight = self.declare_variable('cruise_motor_weight', val=71*9.81)

        wing_battery_weight_ratio = self.declare_variable('wing_battery_weight_ratio', val = 0.)
        wing_weight = self.declare_variable('wing_weight')
        horitontal_stabilizer_weight = self.declare_variable('horizontal_stabilizer_weight')
        vertical_stabilizer_weight = self.declare_variable('vertical_stabilizer_weight')
        fuselage_weight = self.declare_variable('fuselage_weight')
        tail_boom_weight = self.declare_variable('tail_boom_weight')
        misc_weight = self.declare_variable('misc_weight')

        wing_center_of_mass = self.declare_variable('wing_center_of_mass', val = 0.45)
        vertical_stabilizer_center_of_mass = self.declare_variable('vertical_stabilizer_center_of_mass', val = 0.9)
        fuselage_len = self.declare_variable('fuselage_length', val = 7)
        tail_boom_length = self.declare_variable('tail_boom_length', val = 4)
        batt_center_of_mass = self.declare_variable('batt_center_of_mass', val = 0.12)

        fuselage_center_of_mass = 0.5 * fuselage_len
        back_prop_center_of_mass = fuselage_len + 0.25

        wing_center_of_mass = wing_center_of_mass * fuselage_len
        horizontal_stabilizer_center_of_mass = wing_center_of_mass + tail_boom_length
        vertical_stabilizer_center_of_mass = vertical_stabilizer_center_of_mass * tail_boom_length + wing_center_of_mass
        batt_center_of_mass = batt_center_of_mass * fuselage_len

        tail_boom_center_of_mass = wing_center_of_mass + 0.5 * tail_boom_length

        w1 = wing_weight + battery_weight
        w2 = horitontal_stabilizer_weight
        w3 = vertical_stabilizer_weight
        w4 = fuselage_weight + payload_weight
        w5 = cruise_motor_weight
        w6 = (1 - wing_battery_weight_ratio) * battery_weight + misc_weight
        w7 = tail_boom_weight
        wx = w1*wing_center_of_mass + w2*horizontal_stabilizer_center_of_mass + w3*vertical_stabilizer_center_of_mass + w4*fuselage_center_of_mass + \
                w5*back_prop_center_of_mass + w6*batt_center_of_mass + w7*tail_boom_center_of_mass
        center_of_mass = wx / (w1+w2+w3+w4+w5+w6+w7)

        wing_aspect_ratio = self.declare_variable('wing_aspect_ratio')
        W_mean_aerodynamic_chord = self.declare_variable('wing_mean_aerodynamic_chord')
        wing_area = self.declare_variable('wing_area', val=1.)
        wingspan = self.declare_variable('wingspan')
        wing_sweep = self.declare_variable('wing_sweep_angle', val=0)
        wing_taper = self.declare_variable('wing_taper_ratio')
        horizontal_stabilizer_aspect_ratio = self.declare_variable('horizontal_stabilizer_aspect_ratio')
        tail_mean_aerodynamic_chord = self.declare_variable('horizontal_stabilizer_mean_aerodynamic_chord')
        horizontal_stabilizer_area = self.declare_variable('horizontal_stabilizer_area')
        vertical_stabilizer_aspect_ratio = self.declare_variable('vertical_stabilizer_aspect_ratio')
        vertical_stabilizer_mean_aerodynamic_chord = self.declare_variable('vertical_stabilizer_mean_aerodynamic_chord')
        vertical_stabilizer_area = self.declare_variable('vertical_stabilizer_area')

        dihedral = self.declare_variable('dihedral', val=2/180*np.pi)

        C_l_alpha = 2 * np.pi

        C_L_alpha_w = C_l_alpha/(1+C_l_alpha/(np.pi*wing_aspect_ratio))
        C_L_alphat_h = C_l_alpha/(1+C_l_alpha/(np.pi*horizontal_stabilizer_aspect_ratio))
        C_L_alphat_v = C_l_alpha/(1+C_l_alpha/(np.pi*vertical_stabilizer_aspect_ratio))

        deps_dalpha = 0.45
        eta_h = 0.9
        dsigma_dbeta = (vertical_stabilizer_area / wing_area) * 3.06 / (1 + csdl.cos(wing_sweep)) + 0.009 * wing_aspect_ratio - 0.276
        eta_v = 0.9

        C_L_alpha_h = C_L_alphat_h * (1 - deps_dalpha) * eta_h
        C_L_alpha_v = C_L_alphat_v * (1 + dsigma_dbeta) * eta_v

        W_ac = wing_center_of_mass - W_mean_aerodynamic_chord / 4
        horizontal_stabilizer_ac = horizontal_stabilizer_center_of_mass - tail_mean_aerodynamic_chord / 4
        vertical_stabilizer_ac = vertical_stabilizer_center_of_mass - vertical_stabilizer_mean_aerodynamic_chord / 4

        x_np = (C_L_alpha_w * wing_area * W_ac + C_L_alpha_h * horizontal_stabilizer_area * horizontal_stabilizer_ac) / \
                (C_L_alpha_w * wing_area + C_L_alpha_h * horizontal_stabilizer_area)

        SM = (x_np - center_of_mass) / W_mean_aerodynamic_chord

        L_h = horizontal_stabilizer_ac - W_ac
        c_h = L_h * horizontal_stabilizer_area / W_mean_aerodynamic_chord / wing_area

        L_v = vertical_stabilizer_ac - W_ac
        c_v = L_v * vertical_stabilizer_area / wingspan / wing_area

        horizontal_stabilizer_ratio = horizontal_stabilizer_area / wing_area
        vertical_stabilizer_ratio = vertical_stabilizer_area / wing_area

        C_Lalpha = C_L_alpha_w + C_L_alpha_h * horizontal_stabilizer_ratio
        C_mq = -2 * eta_h * C_L_alphat_h * horizontal_stabilizer_ratio * ((horizontal_stabilizer_ac - center_of_mass) / W_mean_aerodynamic_chord)**2
        C_malpha = C_L_alpha_w * ((center_of_mass-W_ac)/W_mean_aerodynamic_chord) - C_L_alpha_h * horizontal_stabilizer_ratio * ((horizontal_stabilizer_ac-center_of_mass)/W_mean_aerodynamic_chord)

        C_lbeta = -0.66 * dihedral
        C_lp = -C_Lalpha/12*(1+3*wing_taper)/(1+wing_taper)
        C_nbeta = C_L_alpha_v * vertical_stabilizer_ratio * (vertical_stabilizer_ac - center_of_mass) / wingspan
        C_ybeta = -C_L_alpha_v * vertical_stabilizer_ratio
        C_nr = -2 * C_L_alphat_v * eta_v * vertical_stabilizer_ratio * ((vertical_stabilizer_ac-center_of_mass)/wingspan)**2

        self.register_output('center_of_mass', center_of_mass)
        self.register_output('np', x_np)
        self.register_output('SM', SM)
        self.register_output('hvol_coeff', c_h)
        self.register_output('vvol_coeff', c_v)
        self.register_output('C_Lalpha', C_Lalpha)
        self.register_output('C_mq', C_mq)
        self.register_output('C_malpha', C_malpha)
        self.register_output('C_lbeta', C_lbeta)
        self.register_output('C_lp', C_lp)
        self.register_output('C_nbeta', C_nbeta)
        self.register_output('C_ybeta', C_ybeta)
        self.register_output('C_nr', C_nr)

        self.register_output('dist_between_horizontal_stabilizer_and_fuse', horizontal_stabilizer_center_of_mass-fuselage_len)
