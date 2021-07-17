import omtools.api as ot
import numpy as np

class UpdateGeoParameters(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    wingpsan = self.declare_input('wingspan', val=11)
    wing_mac = self.declare_input('wing_mac', val=1)
    wing_taper_ratio = self.declare_input('wing_taper_ratio', val=0.45)
    wing_thick = self.declare_input('wing_t', val=0.12)
    fuse_length = self.declare_input('fuse_length', val=10.)
    fuse_width = self.declare_input('fuse_width', val=1.5)
    fuse_height = self.declare_input('fuse_height', val=1.5)
    tail_boom_length = self.declare_input('tail_boom_length', val=2.)
    tail_boom_radius = self.declare_input('tail_boom_radius', val=0.2)

    sweep_angle = self.declare_input('sweep_angle', val=0.)

    # cl_max = self.declare_input('cl_max', val=1.77)

    htail_span = self.declare_input('htail_span', val=4.)
    htail_mac = self.declare_input('htail_mac', val=4.)
    htail_taper_ratio = self.declare_input('htail_taper_ratio', val=0.45)
    htail_thick = self.declare_input('htail_t', val=0.5)
    vtail_span = self.declare_input('vtail_span', val=4.)
    vtail_mac = self.declare_input('vtail_mac', val=4.)
    vtail_taper_ratio = self.declare_input('vtail_taper_ratio', val=0.45)
    vtail_thick = self.declare_input('vtail_t', val=0.5)
    fuse_length = self.declare_input('fuse_length', val=10.)
    fuse_width = self.declare_input('fuse_width', val=1.5)
    fuse_height = self.declare_input('fuse_height', val=1.5)
    

    wing_area = wingpsan*wing_mac
    ar = wingpsan**2/wing_area
    wing_chord_root = wing_mac*2/(1 + wing_taper_ratio)
    wing_chord_tip = wing_chord_root*wing_taper_ratio
    wing_t_c = wing_thick/wing_mac
    wing_wetted = wing_area*2.05
    
    htail_area = htail_span*htail_mac
    htail_ar = htail_span**2/htail_area
    htail_chord_root = htail_mac*2/(1 + htail_taper_ratio)
    htail_chord_tip = htail_chord_root*htail_taper_ratio
    htail_t_c = htail_thick/htail_mac
    htail_wetted = htail_area*2.05
    vtail_area = vtail_span*vtail_mac
    vtail_ar = vtail_span**2/vtail_area
    vtail_chord_root = vtail_mac*2/(1 + vtail_taper_ratio)
    vtail_chord_tip = vtail_chord_root*vtail_taper_ratio
    vtail_t_c = vtail_thick/vtail_mac
    vtail_wetted = vtail_area*2.05

    tail_boom_area = 2*np.pi*tail_boom_radius*tail_boom_length

    fuse_wetted = fuse_length*fuse_width*2 + fuse_length*fuse_height*2

    # cL_max = 0.9*cl_max*ot.cos(sweep_angle)
    fuse_depth = 1/2 * (fuse_width + fuse_height)

    self.register_output('wing_area', wing_area)
    self.register_output('ar', ar)
    self.register_output('wing_chord_root', wing_chord_root)
    self.register_output('wing_chord_tip', wing_chord_tip)
    self.register_output('wing_t_c', wing_t_c)
    self.register_output('wing_wetted', wing_wetted)

    self.register_output('htail_area', htail_area)
    self.register_output('htail_ar', htail_ar)
    self.register_output('htail_chord_root', htail_chord_root)
    self.register_output('htail_chord_tip', htail_chord_tip)
    self.register_output('htail_t_c', htail_t_c)
    self.register_output('htail_wetted', htail_wetted)
  
    self.register_output('vtail_area', vtail_area)
    self.register_output('vtail_ar', vtail_ar)
    self.register_output('vtail_chord_root', vtail_chord_root)
    self.register_output('vtail_chord_tip', vtail_chord_tip)
    self.register_output('vtail_t_c', vtail_t_c)
    self.register_output('vtail_wetted', vtail_wetted)

    self.register_output('fuse_depth', fuse_depth)

    self.register_output('tail_boom_area', tail_boom_area)

    self.register_output('fuse_wetted', fuse_wetted)

    # self.register_output('cL_max', cL_max)
