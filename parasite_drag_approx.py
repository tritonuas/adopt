from re import T
from networkx.algorithms.clique import find_cliques
from networkx.algorithms.operators.product import _nodes_cross_edges
import openmdao.api as om
import omtools.api as ot
import numpy as np



def lam_func(air_density, air_viscosity, velocity_cruise, x):

    cf_lam_comp = 1.328/(air_density*velocity_cruise*x/air_viscosity)**(1/2)
    return cf_lam_comp

def turb_func(air_density, air_viscosity, velocity_cruise, x, a):
    
    M = velocity_cruise/a

    cf_turb_comp = 0.455/(ot.log10(air_density*velocity_cruise*x/air_viscosity)**2.58 \
        *(1 + 0.144*M**2)**0.65)
    return cf_turb_comp


def calc_cf(air_density, air_viscosity, length, velocity_cruise, a):
    k = 0.7e-5
    RE_CRIT_TURB = 38.21*(length/k)**1.053

    x_crit_turb = RE_CRIT_TURB*air_viscosity/(velocity_cruise*air_density)

    # Can't implement if statements using OM Tools
    # if length > x_crit_turb:
    #     cf_lam_comp = lam_func(air_density, air_viscosity, x_crit_turb, velocity_cruise)
    #     cf_turb_comp = turb_func(air_density, air_viscosity, velocity_cruise, length, a)

    #     cf = (cf_lam_comp*x_crit_turb + \
    #         cf_turb_comp*(length-x_crit_turb)) \
    #         /length
    # else:
    #     cf = lam_func(air_density, air_viscosity, length, velocity_cruise)
    percent_lam = 0.

    cf_lam_comp = lam_func(air_density, air_viscosity, length*percent_lam, velocity_cruise)
    cf_turb_comp = turb_func(air_density, air_viscosity, velocity_cruise, length, a)

    cf = (cf_lam_comp*percent_lam + cf_turb_comp*(length*(1-percent_lam)))/length
    # return cf
    return cf_turb_comp


def FF_wing(x_over_c, t_over_c, M, sweep_angle):
    FF = ((1 + 0.6/x_over_c*t_over_c + 100*t_over_c**4)*(1.34*M**0.18*(ot.cos(sweep_angle))**0.28))
    return FF


def FF_fuse(f):
    FF = 1 + 60/f**3 + f/400
    return FF


def FF_nacelle(f):
    FF = 1 + 0.35/f
    return FF


def f_coeff(length, max_cross_area):
    f = length/(4/np.pi*max_cross_area)**(1/2)
    return f
    


class ParasiteDragApprox(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    air_density = self.declare_input('air_density', val=1.225)
    air_viscosity = self.declare_input('air_viscosity', val=1.789e-5)
    a = self.declare_input('a', val=343)

    velocity_cruise = self.declare_input('velocity_cruise', val=50.)

    wing_area = self.declare_input('wing_area', val=11.)
    wing_mac = self.declare_input('wing_mac', val=1.)
    fuse_wetted = self.declare_input('fuse_wetted', val=50.)
    fuse_width = self.declare_input('fuse_width', val=2.)
    fuse_height = self.declare_input('fuse_height', val=2.)
    fuse_length = self.declare_input('fuse_length', val=5.)
    nose_wetted = self.declare_input('nose_wetted', val=1e-5)
    nose_length = self.declare_input('nose_length', val=1.)
    htail_wetted = self.declare_input('htail_wetted', val=2.)
    htail_mac = self.declare_input('htail_mac', val=0.5)
    vtail_wetted = self.declare_input('vtail_wetted', val=2.)
    vtail_mac = self.declare_input('vtail_mac', val=0.5)
    tail_boom_area = self.declare_input('tail_boom_area', val=5)
    tail_boom_radius = self.declare_input('tail_boom_radius', val=0.5)
    tail_boom_length = self.declare_input('tail_boom_length', val=2.)

    sweep_angle = self.declare_input('sweep_angle', val=0.)

    M = velocity_cruise/a

    cf_wing = calc_cf(air_density, air_viscosity, wing_mac, velocity_cruise, a)
    cf_fuse = calc_cf(air_density, air_viscosity, fuse_length, velocity_cruise, a)
    cf_nose = calc_cf(air_density, air_viscosity, nose_length, velocity_cruise, a)
    cf_htail = calc_cf(air_density, air_viscosity, htail_mac, velocity_cruise, a)
    cf_vtail = calc_cf(air_density, air_viscosity, vtail_mac, velocity_cruise, a)
    cf_tail_boom = calc_cf(air_density, air_viscosity, tail_boom_length, velocity_cruise, a)


    M = velocity_cruise/a

    t_over_c = 0.12
    x_over_c = 0.4
    FFWing = FF_wing(x_over_c, t_over_c, M, sweep_angle)

    t_over_c = 0.15
    x_over_c = 0.4
    FFHTail = FF_wing(x_over_c, t_over_c, M, sweep_angle)
    FFVTail = FF_wing(x_over_c, t_over_c, M, sweep_angle)

    fuse_cross_section_area = fuse_width*fuse_height
    fFuse = f_coeff(fuse_length, fuse_cross_section_area)

    FFFuse = FF_fuse(fFuse)
    nose_cross_section_area = fuse_cross_section_area
    fNose = f_coeff(nose_length, nose_cross_section_area)

    FFNose = FF_fuse(fNose)
    tail_boom_cross_section_area = tail_boom_radius

    fTailBoom = f_coeff(tail_boom_length, tail_boom_cross_section_area)
    FFTailBoom = FF_fuse(fTailBoom)

    # Q = 1.0     # interference factor
    Q = 1.1     # interference factor

    areaRatioWing = 2*wing_area/wing_area
    areaRatioFuse = fuse_wetted/wing_area
    areaRatioNose = nose_wetted/wing_area
    areaRatioHTail = htail_wetted/wing_area
    areaRatioVTail = vtail_wetted/wing_area
    areaRatioTailBoom = tail_boom_area/wing_area

    cD0 = Q*(cf_wing*FFWing*areaRatioWing + cf_fuse*FFFuse*areaRatioFuse + \
        cf_nose*FFNose*areaRatioNose + cf_htail*FFHTail*areaRatioHTail + \
        cf_vtail*FFVTail*areaRatioVTail + cf_tail_boom*FFTailBoom*areaRatioTailBoom)

    wing_term = cf_wing*FFWing*areaRatioWing
    htail_term = cf_htail*FFHTail*areaRatioHTail
    vtail_term = cf_vtail*FFVTail*areaRatioVTail
    fuse_term = cf_fuse*FFFuse*areaRatioFuse
    nose_term = cf_nose*FFNose*areaRatioNose
    tail_boom_term = 2*cf_tail_boom*FFTailBoom*areaRatioTailBoom

    self.register_output('cD0', cD0)
    # self.register_output('FFWing', FFWing)
    self.register_output('cf_wing', cf_wing)
    self.register_output('FFFuse', FFFuse)
    self.register_output('wing_term', wing_term)
    self.register_output('htail_term', htail_term)
    self.register_output('vtail_term', vtail_term)
    self.register_output('fuse_term', fuse_term)
    self.register_output('nose_term', nose_term)
    self.register_output('tail_boom_term', tail_boom_term)
    
