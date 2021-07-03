import omtools.api as ot
import numpy as np

class CalcStatStab(ot.Group):

    def initialize(self):
        pass

    def setup(self):
        payload_weight = self.declare_input('payload_weight', val=400*9.81)
        battery_weight = self.declare_input('battery_weight_cruise', val=300*9.81)
        battery_weight_vtol = self.declare_input('battery_weight_vtol', val=500*9.81)
        vtol_motor_weight = self.declare_input('vtol_motor_weight', val=25*10*9.81)
        cruise_motor_weight = self.declare_input('cruise_motor_weight', val=71*9.81)

        W_bwing = self.declare_input('wing_battery_weight_ratio', val = 0.25)
        W_wing = self.declare_input('wing_weight')
        W_htail = self.declare_input('htail_weight')
        W_vtail = self.declare_input('vtail_weight')
        W_fuse = self.declare_input('fuse_weight')
        W_tailb = self.declare_input('tail_boom_weight')
        misc_weight = self.declare_input('misc_weight')

        wing_cg = self.declare_input('wing_cg', val = 0.45)
        vtail_cg = self.declare_input('vtail_cg', val = 0.9)
        fuse_len = self.declare_input('fuse_length', val = 7)
        tail_boom_length = self.declare_input('tail_boom_length', val = 4)
        batt_cg = self.declare_input('batt_cg', val = 0.12)

        fuse_cg = 0.5 * fuse_len
        back_prop_cg = fuse_len + 0.25

        wing_cg = wing_cg * fuse_len
        htail_cg = wing_cg + tail_boom_length
        vtail_cg = vtail_cg * tail_boom_length + wing_cg
        batt_cg = batt_cg * fuse_len

        tail_boom_cg = wing_cg + 0.5 * tail_boom_length

        w1 = W_wing + (battery_weight + battery_weight_vtol) * W_bwing + 8 / 10 * vtol_motor_weight
        w2 = W_htail + 2 / 10 * vtol_motor_weight
        w3 = W_vtail
        w4 = W_fuse + payload_weight
        w5 = cruise_motor_weight
        w6 = (1 - W_bwing) * (battery_weight + battery_weight_vtol) + misc_weight
        w7 = W_tailb
        wx = w1*wing_cg + w2*htail_cg + w3*vtail_cg + w4*fuse_cg + \
                w5*back_prop_cg + w6*batt_cg + w7*tail_boom_cg
        cg = wx / (w1+w2+w3+w4+w5+w6+w7)

        W_ar = self.declare_input('ar')
        W_mac = self.declare_input('wing_mac')
        S_wing = self.declare_input('wing_area', val=11.)
        wingspan = self.declare_input('wingpsan')
        wing_sweep = self.declare_input('sweep_angle', val=0)
        wing_taper = self.declare_input('wing_taper_ratio')
        htail_ar = self.declare_input('htail_ar')
        tail_mac = self.declare_input('htail_mac')
        S_htail = self.declare_input('htail_area')
        vtail_ar = self.declare_input('vtail_ar')
        vtail_mac = self.declare_input('vtail_mac')
        S_vtail = self.declare_input('vtail_area')

        dihedral = self.declare_input('dihedral', val=2/180*np.pi)

        C_l_alpha = 2 * np.pi

        C_L_alpha_w = C_l_alpha/(1+C_l_alpha/(np.pi*W_ar))
        C_L_alphat_h = C_l_alpha/(1+C_l_alpha/(np.pi*htail_ar))
        C_L_alphat_v = C_l_alpha/(1+C_l_alpha/(np.pi*vtail_ar))

        deps_dalpha = 0.45
        eta_h = 0.9
        dsigma_dbeta = (S_vtail / S_wing) * 3.06 / (1 + ot.cos(wing_sweep)) + 0.009 * W_ar - 0.276
        eta_v = 0.9

        C_L_alpha_h = C_L_alphat_h * (1 - deps_dalpha) * eta_h
        C_L_alpha_v = C_L_alphat_v * (1 + dsigma_dbeta) * eta_v

        W_ac = wing_cg - W_mac / 4
        htail_ac = htail_cg - tail_mac / 4
        vtail_ac = vtail_cg - vtail_mac / 4

        x_np = (C_L_alpha_w * S_wing * W_ac + C_L_alpha_h * S_htail * htail_ac) / \
                (C_L_alpha_w * S_wing + C_L_alpha_h * S_htail)

        SM = (x_np - cg) / W_mac

        L_h = htail_ac - W_ac
        c_h = L_h * S_htail / W_mac / S_wing

        L_v = vtail_ac - W_ac
        c_v = L_v * S_vtail / wingspan / S_wing

        htail_ratio = S_htail / S_wing
        vtail_ratio = S_vtail / S_wing

        C_Lalpha = C_L_alpha_w + C_L_alpha_h * htail_ratio
        C_mq = -2 * eta_h * C_L_alphat_h * htail_ratio * ((htail_ac - cg) / W_mac)**2
        C_malpha = C_L_alpha_w * ((cg-W_ac)/W_mac) - C_L_alpha_h * htail_ratio * ((htail_ac-cg)/W_mac)

        C_lbeta = -0.66 * dihedral
        C_lp = -C_Lalpha/12*(1+3*wing_taper)/(1+wing_taper)
        C_nbeta = C_L_alpha_v * vtail_ratio * (vtail_ac - cg) / wingspan
        C_ybeta = -C_L_alpha_v * vtail_ratio
        C_nr = -2 * C_L_alphat_v * eta_v * vtail_ratio * ((vtail_ac-cg)/wingspan)**2

        self.register_output('cg', cg)
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

        self.register_output('dist_between_htail_and_fuse', htail_cg-fuse_len)
