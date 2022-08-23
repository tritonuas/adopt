import csdl
import numpy as np

class GeometryModel(csdl.Model):
  
  def initialize(self):
    pass

  def define(self):
    wingpsan = self.declare_variable('wing_span', val=11)
    wing_mean_aerodynamic_chord = self.declare_variable('wing_mean_aerodynamic_chord', val=1)
    wing_taper_ratio = self.declare_variable('wing_taper_ratio', val=0.45)
    wing_thickness = self.declare_variable('wing_thickness', val=0.12)
    fuselage_length = self.declare_variable('fuselage_length', val=10.)
    fuselage_width = self.declare_variable('fuselage_width', val=1.5)
    fuselage_height = self.declare_variable('fuselage_height', val=1.5)
    tail_boom_length = self.declare_variable('tail_boom_length', val=2.)
    tail_boom_radius = self.declare_variable('tail_boom_radius', val=0.2)

    wing_sweep_angle = self.declare_variable('wing_sweep_angle', val=0.)

    # cl_max = self.declare_variable('cl_max', val=1.77)

    horizontal_stabilizer_span = self.declare_variable('horizontal_stabilizer_span', val=4.)
    horizontal_stabilizer_mean_aerodynamic_chord = self.declare_variable('horizontal_stabilizer_mean_aerodynamic_chord', val=4.)
    horizontal_stabilizer_taper_ratio = self.declare_variable('horizontal_stabilizer_taper_ratio', val=0.45)
    horizontal_stabilizer_thickness = self.declare_variable('horizontal_stabilizer_thickness', val=0.5)
    vertical_stabilizer_span = self.declare_variable('vertical_stabilizer_span', val=4.)
    vertical_stabilizer_mean_aerodynamic_chord = self.declare_variable('vertical_stabilizer_mean_aerodynamic_chord', val=4.)
    vertical_stabilizer_taper_ratio = self.declare_variable('vertical_stabilizer_taper_ratio', val=0.45)
    vertical_stabilizer_thickness = self.declare_variable('vertical_stabilizer_thickness', val=0.5)
    fuselage_length = self.declare_variable('fuselage_length', val=10.)
    fuselage_width = self.declare_variable('fuselage_width', val=1.5)
    fuselage_height = self.declare_variable('fuselage_height', val=1.5)
    

    wing_area = wingpsan*wing_mean_aerodynamic_chord
    wing_aspect_ratio = wingpsan**2/wing_area
    wing_chord_root = wing_mean_aerodynamic_chord*2/(1 + wing_taper_ratio)
    wing_chord_tip = wing_chord_root*wing_taper_ratio
    wing_thickness_to_chord_ratio = wing_thickness/wing_mean_aerodynamic_chord
    wing_wetted = wing_area*2.05
    
    horizontal_stabilizer_area = horizontal_stabilizer_span*horizontal_stabilizer_mean_aerodynamic_chord
    horizontal_stabilizer_aspect_ratio = horizontal_stabilizer_span**2/horizontal_stabilizer_area
    horizontal_stabilizer_chord_root = horizontal_stabilizer_mean_aerodynamic_chord*2/(1 + horizontal_stabilizer_taper_ratio)
    horizontal_stabilizer_chord_tip = horizontal_stabilizer_chord_root*horizontal_stabilizer_taper_ratio
    horizontal_stabilizer_thickness_to_chord_ratio = horizontal_stabilizer_thickness/horizontal_stabilizer_mean_aerodynamic_chord
    horizontal_stabilizer_wetted = horizontal_stabilizer_area*2.05
    vertical_stabilizer_area = vertical_stabilizer_span*vertical_stabilizer_mean_aerodynamic_chord
    vertical_stabilizer_aspect_ratio = vertical_stabilizer_span**2/vertical_stabilizer_area
    vertical_stabilizer_chord_root = vertical_stabilizer_mean_aerodynamic_chord*2/(1 + vertical_stabilizer_taper_ratio)
    vertical_stabilizer_chord_tip = vertical_stabilizer_chord_root*vertical_stabilizer_taper_ratio
    vertical_stabilizer_thickness_to_chord_ratio = vertical_stabilizer_thickness/vertical_stabilizer_mean_aerodynamic_chord
    vertical_stabilizer_wetted = vertical_stabilizer_area*2.05

    tail_boom_area = 2*np.pi*tail_boom_radius*tail_boom_length

    fuselage_wetted = fuselage_length*fuselage_width*2 + fuselage_length*fuselage_height*2

    # cL_max = 0.9*cl_max*csdl.cos(wing_sweep_angle)
    fuselage_depth = 1/2 * (fuselage_width + fuselage_height)

    self.register_output('wing_area', wing_area)
    self.register_output('wing_aspect_ratio', wing_aspect_ratio)
    self.register_output('wing_chord_root', wing_chord_root)
    self.register_output('wing_chord_tip', wing_chord_tip)
    self.register_output('wing_thickness_to_chord_ratio', wing_thickness_to_chord_ratio)
    self.register_output('wing_wetted', wing_wetted)

    self.register_output('horizontal_stabilizer_area', horizontal_stabilizer_area)
    self.register_output('horizontal_stabilizer_aspect_ratio', horizontal_stabilizer_aspect_ratio)
    self.register_output('horizontal_stabilizer_chord_root', horizontal_stabilizer_chord_root)
    self.register_output('horizontal_stabilizer_chord_tip', horizontal_stabilizer_chord_tip)
    self.register_output('horizontal_stabilizer_thickness_to_chord_ratio', horizontal_stabilizer_thickness_to_chord_ratio)
    self.register_output('horizontal_stabilizer_wetted', horizontal_stabilizer_wetted)
  
    self.register_output('vertical_stabilizer_area', vertical_stabilizer_area)
    self.register_output('vertical_stabilizer_aspect_ratio', vertical_stabilizer_aspect_ratio)
    self.register_output('vertical_stabilizer_chord_root', vertical_stabilizer_chord_root)
    self.register_output('vertical_stabilizer_chord_tip', vertical_stabilizer_chord_tip)
    self.register_output('vertical_stabilizer_thickness_to_chord_ratio', vertical_stabilizer_thickness_to_chord_ratio)
    self.register_output('vertical_stabilizer_wetted', vertical_stabilizer_wetted)

    self.register_output('fuselage_depth', fuselage_depth)

    self.register_output('tail_boom_area', tail_boom_area)

    self.register_output('fuselage_wetted', fuselage_wetted)

    # self.register_output('cL_max', cL_max)
