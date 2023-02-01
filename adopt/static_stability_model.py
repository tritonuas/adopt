import csdl
import numpy as np

class StaticStabilityModel(csdl.Model):

    def initialize(self):
        pass

    def define(self):
        payload_weight = self.declare_variable('payload_weight', val=6.3*9.81)
        battery_weight = self.declare_variable('battery_weight', val=22.2)
        motor_weight = self.declare_variable('motor_weight', val=1*9.81)

        wing_battery_weight_ratio = self.declare_variable('wing_battery_weight_ratio', val = 0.)
        wing_weight = self.declare_variable('wing_weight')
        horitontal_stabilizer_weight = self.declare_variable('horizontal_stabilizer_weight')
        vertical_stabilizer_weight = self.declare_variable('vertical_stabilizer_weight')
        fuselage_weight = self.declare_variable('fuselage_weight')
        tail_boom_weight = self.declare_variable('tail_boom_weight')
        gross_weight = self.declare_variable('gross_weight')

        wing_center_of_mass = self.declare_variable('wing_center_of_mass', val = 0.45*1.1)
        # vertical_stabilizer_center_of_mass = self.declare_variable('vertical_stabilizer_center_of_mass', val = 1.1+1.)
        # horizontal_stabilizer_center_of_mass = self.declare_variable('horizontal_stabilizer_center_of_mass', val = 1.1+1.)
        fuselage_center_of_mass = self.declare_variable('fuselage_center_of_mass', 0.25*1.1)
        # tail_boom_center_of_mass = self.declare_variable('tail_boom_center_of_mass', val=1.1+0.5)
        battery_center_of_mass = self.declare_variable('battery_center_of_mass', val = 0.15)
        motor_center_of_mass = self.declare_variable('motor_center_of_mass', val=-0.02)
        payload_center_of_mass = self.declare_variable('payload_center_of_mass', val=0.5)

        fuselage_length = self.declare_variable('fuselage_length', val=1.1)
        tail_boom_length = self.declare_variable('tail_boom_length', val=1.)
        tail_boom_center_of_mass = fuselage_length + 0.5*tail_boom_length
        horizontal_stabilizer_center_of_mass = fuselage_length + tail_boom_length*0.9
        vertical_stabilizer_center_of_mass = fuselage_length + tail_boom_length*0.9

        center_of_mass = (wing_weight*wing_center_of_mass + fuselage_weight*fuselage_center_of_mass + tail_boom_weight*tail_boom_center_of_mass + \
            horitontal_stabilizer_weight*horizontal_stabilizer_center_of_mass + vertical_stabilizer_weight*vertical_stabilizer_center_of_mass + \
            battery_weight*battery_center_of_mass + motor_weight*motor_center_of_mass + payload_weight*payload_center_of_mass)/gross_weight


        wing_aspect_ratio = self.declare_variable('wing_aspect_ratio')
        W_mean_aerodynamic_chord = self.declare_variable('wing_mean_aerodynamic_chord')
        wing_area = self.declare_variable('wing_area', val=1.)
        wing_span = self.declare_variable('wing_span')
        wing_sweep = self.declare_variable('wing_sweep_angle', val=0)
        wing_taper = self.declare_variable('wing_taper_ratio')
        horizontal_stabilizer_aspect_ratio = self.declare_variable('horizontal_stabilizer_aspect_ratio')
        tail_mean_aerodynamic_chord = self.declare_variable('horizontal_stabilizer_mean_aerodynamic_chord')
        horizontal_stabilizer_area = self.declare_variable('horizontal_stabilizer_area')
        vertical_stabilizer_aspect_ratio = self.declare_variable('vertical_stabilizer_aspect_ratio')
        vertical_stabilizer_mean_aerodynamic_chord = self.declare_variable('vertical_stabilizer_mean_aerodynamic_chord')
        vertical_stabilizer_area = self.declare_variable('vertical_stabilizer_area')

        wing_dihedral = self.declare_variable('wing_dihedral', val=2/180*np.pi)

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
        c_v = L_v * vertical_stabilizer_area / wing_span / wing_area

        horizontal_stabilizer_ratio = horizontal_stabilizer_area / wing_area
        vertical_stabilizer_ratio = vertical_stabilizer_area / wing_area

        C_Lalpha = C_L_alpha_w + C_L_alpha_h * horizontal_stabilizer_ratio
        C_mq = -2 * eta_h * C_L_alphat_h * horizontal_stabilizer_ratio * ((horizontal_stabilizer_ac - center_of_mass) / W_mean_aerodynamic_chord)**2
        C_malpha = C_L_alpha_w * ((center_of_mass-W_ac)/W_mean_aerodynamic_chord) - C_L_alpha_h * horizontal_stabilizer_ratio * ((horizontal_stabilizer_ac-center_of_mass)/W_mean_aerodynamic_chord)

        C_lbeta = -0.66 * wing_dihedral
        C_lp = -C_Lalpha/12*(1+3*wing_taper)/(1+wing_taper)
        C_nbeta = C_L_alpha_v * vertical_stabilizer_ratio * (vertical_stabilizer_ac - center_of_mass) / wing_span
        C_ybeta = -C_L_alpha_v * vertical_stabilizer_ratio
        C_nr = -2 * C_L_alphat_v * eta_v * vertical_stabilizer_ratio * ((vertical_stabilizer_ac-center_of_mass)/wing_span)**2

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
