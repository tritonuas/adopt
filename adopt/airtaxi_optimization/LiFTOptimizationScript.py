'''
OpenMDAO script for design optimization of UAVs for Triton UAS

Authors: Andrew Fletcher, Wilson Li
Date: 4/21/2021
'''

# import necessary packages
import openmdao.api as om
import numpy as np
import matplotlib.pyplot as plt

# import optimization models/systems
from adopt.geometry_model import GeometryModel
from adopt.gross_weight_coupling_model import GrossWeightCouplingModel
from adopt.aerodynamic_outputs_model import AerodynamicOutputsModel
from adopt.range_model import RangeModel
from adopt.static_stability_model import StaticStabilityModel
from adopt.dynamic_stability_model import DynamicStabilityModel

from adopt.calc_desired_spanwise_cl import get_cl_vals

# Problem declaration
prob = om.Problem()

''' Specifying Inputs '''
# Independent Variables Input
# environmental inputs
g = 9.81    # m/s^2
air_density = 1.225 # sea level air density
air_viscosity = 1.789e-5  # sea level air viscosity
a = 343     # m/s   speed of sound

# technology constraint inputs
battery_energy_density = 500*3600  # W-s/kg=J/kg
empty_weight = 1000*g      # N

# mission inputs
payload_weight = 500*g    # N
battery_weight_cruise = 740*g    # N
battery_weight_vtol = 60*g     # N
vtol_motor_weight = 10 * 25 * g     # N
cruise_motor_weight = 71 * g        # N
total_propulsive_efficiency = 0.9685

# Initial Guesses for Design Variables
# wing variables
wing_mean_aerodynamic_chord = 1          # m
wingspan = 11      # m
wing_taper_ratio = 0.45
wing_sweep_angle = 0
dihedral = 0
wing_thickness = 0.12

# tail variables
horizontal_stabilizer_span = 3.
horizontal_stabilizer_mean_aerodynamic_chord = 1.5
horizontal_stabilizer_sweep = 0
horizontal_stabilizer_taper = 0.45
horizontal_stabilizer_t = 0.18

vertical_stabilizer_span = 2.
vertical_stabilizer_mean_aerodynamic_chord = 1.
vertical_stabilizer_sweep = 0
vertical_stabilizer_taper = 0.45
vertical_stabilizer_t = 0.12

# tail boom variables
tail_boom_length = 4
tail_boom_radius = 0.2

# Centers of gravity (% fuselage length)
wing_center_of_mass = 0.45
horizontal_stabilizer_center_of_mass = 0.9
vertical_stabilizer_center_of_mass = 0.9
batt_center_of_mass = 0.12
tailb_center_of_mass = 0.8

# twist distribution? Don't have any spanwise analysis

# TODO tail/tail_boom parameters

# Variables For Constraints
# operating constraints
max_velocity_stall = 35.     # m/s
min_climb_gradient = .12   # m/m
min_climb_speed = 6  # m/s
# cl_max = 1.77
ultimate_load_factor = 4.

# geometric constraints
max_wingspan = 11   # m
fuselage_width = 1.5      # m
fuselage_height = 2.     # m
fuselage_length = 10.     # m

# commented out because we shouldn't need to worry
min_velocity_cruise = 50   #m/s

wing_battery_ratio = 0.5

indep_comp = om.IndepVarComp()

indep_comp.add_output('g', g)
indep_comp.add_output('air_density', air_density)
indep_comp.add_output('air_viscosity', air_viscosity)
indep_comp.add_output('a', a)

indep_comp.add_output('battery_energy_density', battery_energy_density)
# indep_comp.add_output('empty_weight', empty_weight)

indep_comp.add_output('payload_weight', payload_weight)
indep_comp.add_output('battery_weight_cruise', battery_weight_cruise)
indep_comp.add_output('battery_weight_vtol', battery_weight_vtol)
indep_comp.add_output('vtol_motor_weight', vtol_motor_weight)
indep_comp.add_output('cruise_motor_weight', cruise_motor_weight)
indep_comp.add_output('total_propulsive_efficiency', total_propulsive_efficiency)

indep_comp.add_output('wingspan', wingspan)
indep_comp.add_output('wing_mean_aerodynamic_chord', wing_mean_aerodynamic_chord)   # Mean aerodynamic chord
indep_comp.add_output('wing_taper_ratio', wing_taper_ratio)
indep_comp.add_output('wing_sweep_angle', wing_sweep_angle)
indep_comp.add_output('wing_thickness', wing_thickness)

indep_comp.add_output('horizontal_stabilizer_span', horizontal_stabilizer_span)
indep_comp.add_output('horizontal_stabilizer_mean_aerodynamic_chord', horizontal_stabilizer_mean_aerodynamic_chord)
indep_comp.add_output('horizontal_stabilizer_taper', horizontal_stabilizer_taper)
indep_comp.add_output('horizontal_stabilizer_sweep', horizontal_stabilizer_sweep)
indep_comp.add_output('horizontal_stabilizer_thickness', horizontal_stabilizer_t)

indep_comp.add_output('vertical_stabilizer_span', vertical_stabilizer_span)
indep_comp.add_output('vertical_stabilizer_mean_aerodynamic_chord', vertical_stabilizer_mean_aerodynamic_chord)
indep_comp.add_output('vertical_stabilizer_taper', vertical_stabilizer_taper)
indep_comp.add_output('vertical_stabilizer_sweep', vertical_stabilizer_sweep)
indep_comp.add_output('vertical_stabilizer_thickness', vertical_stabilizer_t)

indep_comp.add_output('tail_boom_radius', tail_boom_radius)
indep_comp.add_output('tail_boom_length', tail_boom_length)

indep_comp.add_output('fuselage_width', fuselage_width)
indep_comp.add_output('fuselage_height', fuselage_height)
indep_comp.add_output('fuselage_length', fuselage_length)

indep_comp.add_output('wing_center_of_mass', wing_center_of_mass)
# indep_comp.add_output('horizontal_stabilizer_center_of_mass', horizontal_stabilizer_center_of_mass)
indep_comp.add_output('vertical_stabilizer_center_of_mass', vertical_stabilizer_center_of_mass)
indep_comp.add_output('batt_center_of_mass', batt_center_of_mass)

indep_comp.add_output('wing_battery_weight_ratio', wing_battery_ratio)

indep_comp.add_output('max_velocity_stall', max_velocity_stall)
indep_comp.add_output('min_climb_speed', min_climb_speed)
indep_comp.add_output('min_climb_gradient', min_climb_gradient)
indep_comp.add_output('ultimate_load_factor', ultimate_load_factor)
# indep_comp.add_output('cl_max', cl_max)

''' Rest of the optimization model/system '''
# Add independent variables to the problem
prob.model.add_subsystem('inputs_comp', indep_comp, promotes=['*'])

# Update geometry based on new design variable values
update_geo_params_comp = GeometryModel()
prob.model.add_subsystem('update_geo_params_comp', update_geo_params_comp, promotes=['*'])

# # Calculate gross weight
# gross_weight_comp = CalcGrossWeight()
# prob.model.add_subsystem('gross_weight_comp', gross_weight_comp, promotes=['*'])

# # Approximate oswald efficiency factor
# oswald_eff_approx_comp = OswaldEffApprox()
# prob.model.add_subsystem('oswald_eff_approx_comp', oswald_eff_approx_comp, promotes=['*'])

# # # Calculate parasite drag
# # parasite_drag_comp = ParasiteDragApprox()
# # prob.model.add_subsystem('parasite_drag_comp', parasite_drag_comp, promotes=['*'])

# # # Calculate optimal cl and velocity
# # opt_drag_polar_comp = CalcOptDragPolar()
# # prob.model.add_subsystem('opt_drag_polar_comp', opt_drag_polar_comp, promotes=['*'])

# # Above two groups in a group
# velocity_coupling_group = VelocityCouplingGroup()
# prob.model.add_subsystem('velocity_coupling_group', velocity_coupling_group, promotes=['*'])

# Add coupling group for gross weight
gross_weight_coupling_group = GrossWeightCouplingModel()
prob.model.add_subsystem('gross_weight_coupling_group', gross_weight_coupling_group, promotes=['*'])

# Get static stability outputs
stat_stab_comp = StaticStabilityModel()
prob.model.add_subsystem('stat_stab_comp', stat_stab_comp, promotes = ['*'])

# Get dynamic stability outputs
dyn_stab_comp = DynamicStabilityModel()
prob.model.add_subsystem('dyn_stab_comp', dyn_stab_comp, promotes = ['*'])

# Get aerodynamic outputs
aero_outputs_comp = AerodynamicOutputsModel()
prob.model.add_subsystem('aero_outputs_comp', aero_outputs_comp, promotes=['*'])

# Calculate range
range_comp = RangeModel()
prob.model.add_subsystem('range_comp', range_comp, promotes=['*'])


''' Specifying Design Variables, Objective, and Constraints '''
# # Add design variables to the problem
# prob.model.add_design_var('wingspan')
prob.model.add_design_var('wing_mean_aerodynamic_chord')
prob.model.add_design_var('horizontal_stabilizer_span')
prob.model.add_design_var('horizontal_stabilizer_mean_aerodynamic_chord')
prob.model.add_design_var('vertical_stabilizer_span')
prob.model.add_design_var('vertical_stabilizer_mean_aerodynamic_chord')
prob.model.add_design_var('fuselage_length')
prob.model.add_design_var('battery_weight_cruise')
# prob.model.add_design_var('wing_battery_weight_ratio')
prob.model.add_design_var('wing_center_of_mass')
# prob.model.add_design_var('horizontal_stabilizer_center_of_mass')
prob.model.add_design_var('vertical_stabilizer_center_of_mass')
prob.model.add_design_var('batt_center_of_mass')
# prob.model.add_design_var('tailb_center_of_mass')
prob.model.add_design_var('tail_boom_length')
# prob.model.add_design_var('tail_boom_radius')

# # Add the objective to the problem
prob.model.add_objective('range', scaler=-1)

# # Add the constraints to the problem
# prob.model.add_constraint('wingspan', upper=max_wingspan, lower=0.1)
prob.model.add_constraint('wing_mean_aerodynamic_chord', lower=0.1, upper=10.)
prob.model.add_constraint('horizontal_stabilizer_span', lower=2.5, upper=8.)
prob.model.add_constraint('horizontal_stabilizer_mean_aerodynamic_chord', lower=0.05, upper=4.)
prob.model.add_constraint('horizontal_stabilizer_aspect_ratio', lower=2.5, upper=5.)
prob.model.add_constraint('vertical_stabilizer_span', lower=0.05, upper=6.)
prob.model.add_constraint('vertical_stabilizer_mean_aerodynamic_chord', lower=0.05, upper=3.)
prob.model.add_constraint('vertical_stabilizer_aspect_ratio', lower=1., upper=3.)
prob.model.add_constraint('fuselage_length', lower=4., upper=10.)
prob.model.add_constraint('tail_boom_length', lower=0.001, upper=6.)
# prob.model.add_constraint('tail_boom_radius', lower=1e-4, upper=1.)
# prob.model.add_constraint('velocity_cruise', lower=min_velocity_cruise)
prob.model.add_constraint('velocity_stall', upper=max_velocity_stall)

prob.model.add_constraint('cL', upper=1.25)

# Weights constraints
prob.model.add_constraint('battery_weight_cruise', lower=0., upper=740*g)
# prob.model.add_constraint('wing_battery_weight_ratio', lower=0.01, upper=0.99)
prob.model.add_constraint('wing_center_of_mass', lower=0.3, upper=0.6)
# prob.model.add_constraint('horizontal_stabilizer_center_of_mass', lower=0.8, upper=0.95)
prob.model.add_constraint('vertical_stabilizer_center_of_mass', lower=0.5, upper=0.9)
prob.model.add_constraint('batt_center_of_mass', lower=0.08, upper=0.9)
prob.model.add_constraint('dist_between_horizontal_stabilizer_and_fuse', lower=1.5)

# Static stability constraints
prob.model.add_constraint('SM', lower=0.1, upper=0.2)
prob.model.add_constraint('hvol_coeff', lower=0.55, upper=0.85)
prob.model.add_constraint('vvol_coeff', lower=0.07)
# prob.model.add_constraint('vvol_coeff', lower=0.055, upper=0.06)
prob.model.add_constraint('C_Lalpha', lower=0)
prob.model.add_constraint('C_mq', upper=0)
prob.model.add_constraint('C_malpha', upper=0)
prob.model.add_constraint('C_lp', upper=0)
# prob.model.add_constraint('C_lbeta', upper=0) # dihedral is indepently chosen
prob.model.add_constraint('C_nbeta', lower=0)
prob.model.add_constraint('C_ybeta', upper=0)
prob.model.add_constraint('C_nr', upper=0)

# # Dynamic stability constraints
prob.model.add_constraint('w_sp', lower=4, upper=16.)
prob.model.add_constraint('z_sp', lower=0.16, upper=1.0)
prob.model.add_constraint('w_dr', lower=1)
prob.model.add_constraint('z_dr', lower=0.04)
prob.model.add_constraint('wd_dr', lower=0.1225)
prob.model.add_constraint('t_roll', upper=2.0)

''' Adding Optimizer to the Problem '''
# prob.driver = om.ScipyOptimizeDriver()
# prob.driver.options['optimizer'] = 'SLSQP'
# prob.driver.options['maxiter'] = 20   # How to set max iterations
# prob.model.nonlinear_solver = om.NonlinearBlockGS()
# prob.model.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)

prob.driver = om.pyOptSparseDriver()  # Adding a driver to the problem
prob.driver.options['optimizer'] = 'SNOPT'  # Picking SNOPT
prob.driver.opt_settings['Major feasibility tolerance'] = 1.0e-9
prob.driver.opt_settings['Major optimality tolerance'] =2.e-12

''' Setup and Run Optimization '''
prob.setup()    # Organizes and arranges the systems/models for optimization

# prob.run_model()  # Runs a single iteration
prob.run_driver()   # Runs optimization

''' Post-optimization Optimizations'''


''' Post-optimization functions '''
num_airfoils = 50
wing_cl_dist = get_cl_vals(prob['wingspan'], prob['gross_weight'], air_density, \
    prob['velocity_cruise'], prob['wing_chord_root'], prob['wing_chord_tip'], num_airfoils)


# # How to print out problem outputs
print('wing_area', prob['wing_area'])
print('wingspan', prob['wingspan'])
print('cL: ', prob['cL'])
print('fuselage_length', prob['fuselage_length'])
print('velocity_cruise: ', prob['velocity_cruise'])
print('battery_weight_cruise: ', prob['battery_weight_cruise'])
# print('FFWing: ', prob['FFWing'])
# print('FFFuse: ', prob['FFFuse'])

print()
print('Drag Breakdown')
print('wing_term: ', prob['wing_term'])
print('horizontal_stabilizer_term: ', prob['horizontal_stabilizer_term'])
print('vertical_stabilizer_term: ', prob['vertical_stabilizer_term'])
print('fuselage_term: ', prob['fuselage_term'])
print('nose_term: ', prob['nose_term'])
print('tail_boom_term: ', prob['tail_boom_term'])

print()
print('e: ', prob['oswald_eff_factor'])
print('cd0: ', prob['cD0'])
print('cdi: ', prob['cDi'])
print('L/D: ', prob['l_over_d'])
print('lift: ', prob['lift'])
print('wing_mean_aerodynamic_chord: ', prob['wing_mean_aerodynamic_chord'])
print('range: ', prob['range'])

print('wing_aspect_ratio: ', prob['wing_aspect_ratio'])
print('wing_loading', prob['wing_loading'])

print('velocity_stall: ', prob['velocity_stall'])
print('P/W required climb: ', prob['power_per_weight_req_climb'])
print('thrust required climb: ', prob['thrust_req_climb'])
print('P/W required maneuver: ', prob['power_per_weight_req_maneuver'])
print('thrust required maneuver: ', prob['thrust_req_maneuver'])

print("Wing battery ratio: ", prob['wing_battery_weight_ratio'])

print()
print('Horizaontal and vertical tail sizing')
print("HTail span: ", prob['horizontal_stabilizer_span'])
print("HTail mac: ", prob['horizontal_stabilizer_mean_aerodynamic_chord'])
print("HTail AR: ", prob['horizontal_stabilizer_aspect_ratio'])
print("VTail span: ", prob['vertical_stabilizer_span'])
print("VTail mac: ", prob['vertical_stabilizer_mean_aerodynamic_chord'])
print("VTail AR: ", prob['vertical_stabilizer_aspect_ratio'])
print("Horizontal tail volume coeff: ", prob['hvol_coeff'])
print("Vertical tail volume coeff: ", prob['vvol_coeff'])

print()
print('Weights')
print('gross_weight: ', prob['gross_weight']/g, 'kg')
print('empty_weight', prob['empty_weight']/g, 'kg')
print('wing_weight:', prob['wing_weight']/g, 'kg')
print('horizontal_stabilizer_weight:', prob['horizontal_stabilizer_weight']/g, 'kg')
print('vertical_stabilizer_weight:', prob['vertical_stabilizer_weight']/g, 'kg')
print('fuselage_weight:', prob['fuselage_weight']/g, 'kg')
print('tail_boom_weight:', prob['tail_boom_weight']/g, 'kg')
print('motor_weight:', prob['motor_weight']/g, 'kg')
print('misc_weight:', prob['misc_weight']/g, 'kg')

print()
print('Total length', prob['fuselage_length']*prob['wing_center_of_mass']+prob['tail_boom_length'])
print("Total CG location: ", prob['center_of_mass'])
print('Fuselage length:', prob['fuselage_length'])
print("Wing CG location: ", prob['wing_center_of_mass']*prob['fuselage_length'])
print("Vtail CG location: ", prob['vertical_stabilizer_center_of_mass']*prob['tail_boom_length'] + prob['wing_center_of_mass']*prob['fuselage_length'])
print("Ballast CG location: ", prob['batt_center_of_mass']*prob['fuselage_length'])
print("Tail boom length: ", prob['tail_boom_length'])

print("Neutral point: ", prob['np'])
print("SM: ", prob['SM'])

print()
print('Static stability')
print("C_L_alpha: ", prob['C_Lalpha'])
print("C_m_q: ", prob['C_mq'])
print("C_m_alpha: ", prob['C_malpha'])
print("C_l_beta: ", prob['C_lbeta'])
print("C_l_p: ", prob['C_lp'])
print("C_n_beta: ", prob['C_nbeta'])
print("C_y_beta: ", prob['C_ybeta'])
print("C_n_r: ", prob['C_nr'])

print()
print('Dynamic stability')
print("omega_sp: ", prob['w_sp']**(1/2))
print("zeta_sp: ", prob['z_sp']**(1/2))
print("omega_dr: ", prob['w_dr']**(1/2))
print("zeta_dr: ", prob['z_dr']**(1/2))
print("tau_roll: ", prob['t_roll'])

print()
print('wing_cl_dist', wing_cl_dist)
# prob.check_partials(compact_print=True)

om.n2(prob)

# from adopt.csv import reader
# import matplotlib.pyplot as plt
# import seaborn as sns

# sns.set()
# sns.set_style('darkgrid')

# # Load a CSV File
# def load_csv(filename):
#     dataset = list()
#     with open(filename, 'r') as file:
#         csv_reader = reader(file)
#         for row in csv_reader:
#             if not row: continue
#             dataset.append(row)
#     return dataset

# def interpolate(x1, x2, y1, y2, x):
#     return (y2 - y1) / (x2 - x1) * (x - x1) + y1

# data = load_csv('naca4412.csv')
# alpha_4412 = [float(data[ind][0].strip()) for ind in range(1, len(data))]
# C_l_4412 = [float(data[ind][1].strip()) for ind in range(1, len(data))]
# C_d_4412 = [float(data[ind][2].strip()) for ind in range(1, len(data))]

# data = load_csv('naca0012.csv')
# alpha_0012 = [float(data[ind][0].strip()) for ind in range(1, len(data))]
# C_l_0012 = [float(data[ind][1].strip()) for ind in range(1, len(data))]
# C_d_0012 = [float(data[ind][2].strip()) for ind in range(1, len(data))]

# location = wing_cl_dist[:,0]
# C_l_req = wing_cl_dist[:,1]

# def get_outputs(C_l_airfoil, alpha, C_d_airfoil, C_l_req):
#     twist = []
#     Cds = []
#     for C_l in C_l_req:
#         ind = sorted(C_l_airfoil + [C_l]).index(C_l)
#         twist.append(interpolate(C_l_airfoil[ind-1], C_l_airfoil[ind], \
#                 alpha[ind-1], alpha[ind], C_l))
#         Cds.append(interpolate(C_l_airfoil[ind-1], C_l_airfoil[ind], \
#                 C_d_airfoil[ind-1], C_d_airfoil[ind], C_l))
#     return twist, Cds

# twist_4412, Cds_4412 = get_outputs(C_l_4412, alpha_4412, C_d_4412, C_l_req)
# twist_0012, Cds_0012 = get_outputs(C_l_0012, alpha_0012, C_d_0012, C_l_req)

# cutoff = 36

# plt.subplots(1,2,figsize=(12,8))
# plt.subplot(1,2,1)
# plt.plot(location, twist_4412, marker='o',color='maroon', label='NACA 4412')
# plt.xlabel('Span Location [m]', fontsize=19)
# plt.ylabel(r'Twist Angle [$^{\circ}$]',fontsize=19)
# plt.legend(fontsize=19)
# plt.subplot(1,2,2)
# plt.plot(location, [x/y for (x,y) in zip(C_l_req,Cds_4412)], marker='o',color='maroon', label='NACA 4412')
# plt.xlabel('Span Location [m]', fontsize=19)
# plt.ylabel(r'$C_l/C_d$', fontsize=19)
# plt.tight_layout(rect = [0,0.05,1,0.99])
# plt.legend(fontsize=19)

# plt.subplots(1,2,figsize=(12,8))
# plt.subplot(1,2,1)
# plt.plot(location[:cutoff], twist_4412[:cutoff], marker='o',color='maroon', label='NACA 4412')
# plt.plot(location[cutoff:], twist_0012[cutoff:], marker='o',color='darkcyan', label='NACA 0012')
# plt.xlabel('Span Location [m]', fontsize=19)
# plt.ylabel(r'Twist Angle [$^{\circ}$]',fontsize=19)
# plt.legend(fontsize=19)
# plt.subplot(1,2,2)
# plt.plot(location[:cutoff], [x/y for (x,y) in zip(C_l_req,Cds_4412[:cutoff])], marker='o',color='maroon', label='NACA 4412')
# plt.plot(location[cutoff:], [x/y for (x,y) in zip(C_l_req,Cds_4412[cutoff:])], marker='o',color='darkcyan', label='NACA 0012')
# plt.xlabel('Span Location [m]', fontsize=19)
# plt.ylabel(r'$C_l/C_d$', fontsize=19)
# plt.tight_layout(rect = [0,0.05,1,0.99])
# plt.legend(fontsize=19)

# plt.figure()
# plt.plot(location, C_l_req, marker='o',color='maroon')
# plt.xlabel('Span Location [m]', fontsize=19)
# plt.ylabel(r'$C_l$', fontsize=19)
# plt.tight_layout(rect = [0,0.05,1,0.99])

# plt.show()
