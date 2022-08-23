'''
OpenMDAO script for design optimization of UAVs for Triton UAS

Authors: Andrew Fletcher, Wilson Li, Allyson Chen
'''

# import necessary packages
import csdl
from python_csdl_backend import Simulator
# from csdl_om import Simulator
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem

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
adopt = csdl.Model()

''' Specifying Inputs '''
# Independent Variables Input
# environmental inputs
g = 9.81    # m/s^2
air_density = 1.225 # sea level air density
air_viscosity = 1.789e-5  # sea level air viscosity
a = 343     # m/s   speed of sound

# technology constraint inputs
battery_energy_density = (5200*14.8/1000*3600)/0.4365   # W-s/kg=J/kg
tail_boom_weight = 164.6/1000*g # N
horizontal_stabilizer_weight = 261.3/1000*g # N
vertical_stabilizer_weight = 167.4/1000*g # N
wing_weight = (1410.2+1444.4)/1000*g # N
fuselage_weight = 980/1000*g # N
misc_weight = 200/1000*g # N
landing_gear_weight = 850/1000*g # N
empty_weight = tail_boom_weight+horizontal_stabilizer_weight+vertical_stabilizer_weight+wing_weight+fuselage_weight+misc_weight+landing_gear_weight  # N

# mission inputs
payload_weight = (7.3-0.510)*g    # N
battery_weight_cruise = 436.5*4/1000*g    # N
# battery_weight_vtol = 60*g     # N
# vtol_motor_weight = 10 * 25 * g     # N
cruise_motor_weight = 510/1000*g      # N
total_propulsive_efficiency = 0.9

# Initial Guesses for Design Variables
# wing variables
wing_mean_aerodynamic_chord = 0.5       # m
wing_span = 3.65      # m
wing_span = 3.5      # m
wing_taper_ratio = 0.45
wing_sweep_angle = 0
wing_dihedral = 0
wing_thickness = 0.06

# tail variables
horizontal_stabilizer_span = 0.5
horizontal_stabilizer_mean_aerodynamic_chord = 0.25
horizontal_stabilizer_sweep = 0
horizontal_stabilizer_taper = 0.45
horizontal_stabilizer_thickness = 0.03

vertical_stabilizer_span = 0.25
vertical_stabilizer_mean_aerodynamic_chord = 0.25
vertical_stabilizer_sweep = 0
vertical_stabilizer_taper = 0.45
vertical_stabilizer_thickness = 0.03

# tail boom variables
tail_boom_length = 1
tail_boom_radius = 0.0381

# Centers of gravity (% fuselage length)
wing_center_of_mass = 0.3
horizontal_stabilizer_center_of_mass = 0.95
vertical_stabilizer_center_of_mass = 0.95
battery_center_of_mass = 0.1
tail_boom_center_of_mass = 0.8

# twist distribution? Don't have any spanwise analysis

# TODO tail/tail_boom parameters

# Variables For Constraints
# operating constraints
max_velocity_stall = 15.     # m/s
min_climb_gradient = 0.1     # m/m
min_climb_speed = 1          # m/s
# cl_max = 1.77
ultimate_load_factor = 4.

# geometric constraints
max_wing_span = 3.65    # m
fuselage_width = 0.21      # m
fuselage_height = 0.25     # m
fuselage_length = 1.1      # m

# commented out because we shouldn't need to worry
min_velocity_cruise = 15   #m/s

wing_battery_ratio = 0.

adopt.create_input('g', g)
adopt.create_input('air_density', air_density)
adopt.create_input('air_viscosity', air_viscosity)
adopt.create_input('a', a)

adopt.create_input('battery_energy_density', battery_energy_density)
# adopt.create_input('empty_weight', empty_weight)

adopt.create_input('payload_weight', payload_weight)
adopt.create_input('battery_weight_cruise', battery_weight_cruise)
adopt.create_input('wing_weight', wing_weight)
# adopt.create_input('battery_weight_vtol', battery_weight_vtol)
# adopt.create_input('vtol_motor_weight', vtol_motor_weight)
adopt.create_input('cruise_motor_weight', cruise_motor_weight)
adopt.create_input('total_propulsive_efficiency', total_propulsive_efficiency)

adopt.create_input('wing_span', wing_span)
adopt.create_input('wing_mean_aerodynamic_chord', wing_mean_aerodynamic_chord)   # Mean aerodynamic chord
adopt.create_input('wing_taper_ratio', wing_taper_ratio)
adopt.create_input('wing_sweep_angle', wing_sweep_angle)
adopt.create_input('wing_thickness', wing_thickness)

adopt.create_input('horizontal_stabilizer_span', horizontal_stabilizer_span)
adopt.create_input('horizontal_stabilizer_mean_aerodynamic_chord', horizontal_stabilizer_mean_aerodynamic_chord)
adopt.create_input('horizontal_stabilizer_taper', horizontal_stabilizer_taper)
adopt.create_input('horizontal_stabilizer_sweep', horizontal_stabilizer_sweep)
adopt.create_input('horizontal_stabilizer_thickness', horizontal_stabilizer_thickness)

adopt.create_input('vertical_stabilizer_span', vertical_stabilizer_span)
adopt.create_input('vertical_stabilizer_mean_aerodynamic_chord', vertical_stabilizer_mean_aerodynamic_chord)
adopt.create_input('vertical_stabilizer_taper', vertical_stabilizer_taper)
adopt.create_input('vertical_stabilizer_sweep', vertical_stabilizer_sweep)
adopt.create_input('vertical_stabilizer_thickness', vertical_stabilizer_thickness)

adopt.create_input('tail_boom_radius', tail_boom_radius)
adopt.create_input('tail_boom_length', tail_boom_length)

adopt.create_input('fuselage_width', fuselage_width)
adopt.create_input('fuselage_height', fuselage_height)
adopt.create_input('fuselage_length', fuselage_length)

adopt.create_input('wing_center_of_mass', wing_center_of_mass)
# adopt.create_input('horizontal_stabilizer_center_of_mass', horizontal_stabilizer_center_of_mass)
adopt.create_input('vertical_stabilizer_center_of_mass', vertical_stabilizer_center_of_mass)
adopt.create_input('battery_center_of_mass', battery_center_of_mass)

adopt.create_input('wing_battery_weight_ratio', wing_battery_ratio)

adopt.create_input('max_velocity_stall', max_velocity_stall)
adopt.create_input('min_climb_speed', min_climb_speed)
adopt.create_input('min_climb_gradient', min_climb_gradient)
adopt.create_input('ultimate_load_factor', ultimate_load_factor)
# adopt.create_input('cl_max', cl_max)


''' Rest of the optimization model/system '''
# Add independent variables to the problem
# adopt.add('inputs_comp', indep_comp)

# Update geometry based on new design variable values
geometry_model = GeometryModel()
adopt.add(submodel=geometry_model, name='geometry_model')

# Add coupling group for gross weight
gross_weight_coupling_model = GrossWeightCouplingModel()
adopt.add(submodel=gross_weight_coupling_model, name='gross_weight_coupling_model')

# Get static stability outputs
static_stability_model = StaticStabilityModel()
adopt.add(submodel=static_stability_model, name='static_stability_model')

# Get dynamic stability outputs
dynamic_stability_model = DynamicStabilityModel()
adopt.add(submodel=dynamic_stability_model, name='dynamic_stability_model')

# Get aerodynamic outputs
aerodynamic_outputs_model = AerodynamicOutputsModel()
adopt.add(submodel=aerodynamic_outputs_model, name='aerodynamic_outputs_model')

# Calculate range
range_model = RangeModel()
adopt.add(submodel=range_model, name='range_model')


''' Specifying Design Variables, Objective, and Constraints '''
# # Add design variables to the problem
# adopt.add_design_variable('wing_span')
adopt.add_design_variable('wing_mean_aerodynamic_chord')
adopt.add_design_variable('horizontal_stabilizer_span')
adopt.add_design_variable('horizontal_stabilizer_mean_aerodynamic_chord')
adopt.add_design_variable('vertical_stabilizer_span')
adopt.add_design_variable('vertical_stabilizer_mean_aerodynamic_chord')
adopt.add_design_variable('fuselage_length')
# adopt.add_design_variable('battery_weight_cruise')
# adopt.add_design_variable('wing_battery_weight_ratio')
adopt.add_design_variable('wing_center_of_mass')
# adopt.add_design_variable('horizontal_stabilizer_center_of_mass')
adopt.add_design_variable('vertical_stabilizer_center_of_mass')
adopt.add_design_variable('battery_center_of_mass')
# adopt.add_design_variable('tail_boom_center_of_mass')
adopt.add_design_variable('tail_boom_length')
# adopt.add_design_variable('tail_boom_radius')

# # Add the objective to the problem
# adopt.add_objective('range', scaler=-1)
adopt.add_objective('turning_radius')

# # Add the constraints to the problem
# adopt.add_constraint('wing_span', upper=max_wing_span, lower=0.1)
adopt.add_constraint('wing_mean_aerodynamic_chord', lower=0.25, upper=2.)
adopt.add_constraint('horizontal_stabilizer_span', lower=0.25, upper=1.)
adopt.add_constraint('horizontal_stabilizer_mean_aerodynamic_chord', lower=0.05, upper=0.5)
adopt.add_constraint('horizontal_stabilizer_aspect_ratio', lower=2.5, upper=5.)
adopt.add_constraint('vertical_stabilizer_span', lower=0.05, upper=1.)
adopt.add_constraint('vertical_stabilizer_mean_aerodynamic_chord', lower=0.05, upper=0.5)
adopt.add_constraint('vertical_stabilizer_aspect_ratio', lower=1., upper=3.)
adopt.add_constraint('fuselage_length', lower=1.1, upper=2.)
adopt.add_constraint('tail_boom_length', lower=0.001, upper=1.)
# adopt.add_constraint('tail_boom_radius', lower=1e-4, upper=1.)
adopt.add_constraint('velocity_cruise', lower=min_velocity_cruise)
adopt.add_constraint('velocity_stall', upper=max_velocity_stall)

adopt.add_constraint('cL', upper=1.25)

# Weights constraints
# adopt.add_constraint('battery_weight_cruise', lower=0., upper=740*g)
# adopt.add_constraint('wing_battery_weight_ratio', lower=0.01, upper=0.99)
adopt.add_constraint('wing_center_of_mass', lower=0.25, upper=0.6)
# adopt.add_constraint('horizontal_stabilizer_center_of_mass', lower=0.8, upper=0.95)
adopt.add_constraint('vertical_stabilizer_center_of_mass', lower=0.5, upper=0.95)
adopt.add_constraint('battery_center_of_mass', lower=0.08, upper=0.95)
# adopt.add_constraint('dist_between_horizontal_stabilizer_and_fuse', lower=1.5)

# Static stability constraints
adopt.add_constraint('SM', lower=0.1, upper=0.2)
adopt.add_constraint('hvol_coeff', lower=0.55, upper=0.85)
adopt.add_constraint('vvol_coeff', lower=0.07)
# adopt.add_constraint('vvol_coeff', lower=0.055, upper=0.06)
adopt.add_constraint('C_Lalpha', lower=0)
adopt.add_constraint('C_mq', upper=0)
adopt.add_constraint('C_malpha', upper=0)
adopt.add_constraint('C_lp', upper=0)
# adopt.add_constraint('C_lbeta', upper=0) # wing_dihedral is indepently chosen
adopt.add_constraint('C_nbeta', lower=0)
adopt.add_constraint('C_ybeta', upper=0)
adopt.add_constraint('C_nr', upper=0)

# # Dynamic stability constraints
# adopt.add_constraint('w_sp', lower=4, upper=16.)
# adopt.add_constraint('z_sp', lower=0.16, upper=1.0)
# adopt.add_constraint('w_dr', lower=1)
# adopt.add_constraint('z_dr', lower=0.04)
# adopt.add_constraint('wd_dr', lower=0.1225)
# adopt.add_constraint('t_roll', upper=2.0)

''' Adding Optimizer to the Problem '''
# prob.driver = om.ScipyOptimizeDriver()
# prob.driver.options['optimizer'] = 'SLSQP'
# prob.driver.options['maxiter'] = 20   # How to set max iterations
# adopt.nonlinear_solver = om.NonlinearBlockGS()
# adopt.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)

# prob.driver = om.pyOptSparseDriver()  # Adding a driver to the problem
# prob.driver.options['optimizer'] = 'SNOPT'  # Picking SNOPT
# prob.driver.opt_settings['Major feasibility tolerance'] = 1.0e-9
# prob.driver.opt_settings['Major optimality tolerance'] =2.e-12

''' Setup and Run Optimization '''
sim = Simulator(adopt)
# sim.visualize_implementation()
sim.run()   # Runs model
# sim.check_partials(compact_print=True)
# sim.check_totals(compact_print=True)

optimizer_ = 'mod_opt_slsqp'

if optimizer_ == 'mod_opt_slsqp':
    # Instantiate your problem using the csdl Simulator object and name your problem
    prob = CSDLProblem(problem_name='adopt', simulator=sim)
    # Setup your preferred optimizer (SLSQP) with the Problem object
    optimizer = SLSQP(prob, maxiter=100, ftol=1e-10)
    # Solve your optimization problem
    optimizer.solve()
    # Print results of optimization
    optimizer.print_results()

''' Post-optimization Optimizations'''


''' Post-optimization functions '''
num_airfoils = 50
wing_cl_dist = get_cl_vals(sim['wing_span'], sim['gross_weight'], air_density, \
    sim['velocity_cruise'], sim['wing_chord_root'], sim['wing_chord_tip'], num_airfoils)


# # How to print out simlem outputs
print('wing_area', sim['wing_area'])
print('wing_span', sim['wing_span'])
print('cL: ', sim['cL'])
print('fuselage_length', sim['fuselage_length'])
print('velocity_cruise: ', sim['velocity_cruise'])
print('battery_weight_cruise: ', sim['battery_weight_cruise'])
# print('FFWing: ', sim['FFWing'])
# print('FFFuse: ', sim['FFFuse'])

print()
print('Drag Breakdown')
print('wing_term: ', sim['wing_term'])
print('horizontal_stabilizer_term: ', sim['horizontal_stabilizer_term'])
print('vertical_stabilizer_term: ', sim['vertical_stabilizer_term'])
print('fuselage_term: ', sim['fuselage_term'])
print('nose_term: ', sim['nose_term'])
print('tail_boom_term: ', sim['tail_boom_term'])

print()
print('e: ', sim['oswald_efficiency_factor'])
print('cd0: ', sim['cD0'])
print('cdi: ', sim['cDi'])
print('L/D: ', sim['l_over_d'])
print('lift: ', sim['lift'])
print('wing_mean_aerodynamic_chord: ', sim['wing_mean_aerodynamic_chord'])
print('range: ', sim['range'])
print('turning_radius: ', sim['turning_radius'])

print('wing_aspect_ratio: ', sim['wing_aspect_ratio'])
print('wing_loading', sim['wing_loading'])

print('velocity_stall: ', sim['velocity_stall'])
print('P/W required climb: ', sim['power_per_weight_req_climb'])
print('thrust required climb: ', sim['thrust_req_climb'])
print('P/W required maneuver: ', sim['power_per_weight_req_maneuver'])
print('thrust required maneuver: ', sim['thrust_req_maneuver'])

# print("Wing battery ratio: ", sim['wing_battery_weight_ratio'])

print()
print('Horizaontal and vertical tail sizing')
print("HTail span: ", sim['horizontal_stabilizer_span'])
print("HTail mac: ", sim['horizontal_stabilizer_mean_aerodynamic_chord'])
print("HTail AR: ", sim['horizontal_stabilizer_aspect_ratio'])
print("VTail span: ", sim['vertical_stabilizer_span'])
print("VTail mac: ", sim['vertical_stabilizer_mean_aerodynamic_chord'])
print("VTail AR: ", sim['vertical_stabilizer_aspect_ratio'])
print("Horizontal tail volume coeff: ", sim['hvol_coeff'])
print("Vertical tail volume coeff: ", sim['vvol_coeff'])

print()
print('Weights')
print('gross_weight: ', sim['gross_weight']/g, 'kg')
# print('empty_weight', sim['empty_weight']/g, 'kg')
# print('wing_weight:', sim['wing_weight']/g, 'kg')
# print('horizontal_stabilizer_weight:', sim['horizontal_stabilizer_weight']/g, 'kg')
# print('vertical_stabilizer_weight:', sim['vertical_stabilizer_weight']/g, 'kg')
# print('fuselage_weight:', sim['fuselage_weight']/g, 'kg')
# print('tail_boom_weight:', sim['tail_boom_weight']/g, 'kg')
# # print('motor_weight:', sim['motor_weight']/g, 'kg')
# print('misc_weight:', sim['misc_weight']/g, 'kg')

print()
print('Total length', sim['fuselage_length']*sim['wing_center_of_mass']+sim['tail_boom_length'])
print("Total CG location: ", sim['center_of_mass'])
print('Fuselage length:', sim['fuselage_length'])
print("Wing CG location: ", sim['wing_center_of_mass']*sim['fuselage_length'])
print("Vtail CG location: ", sim['vertical_stabilizer_center_of_mass']*sim['tail_boom_length'] + sim['wing_center_of_mass']*sim['fuselage_length'])
print("Ballast CG location: ", sim['battery_center_of_mass']*sim['fuselage_length'])
print("Tail boom length: ", sim['tail_boom_length'])

print("Neutral point: ", sim['np'])
print("SM: ", sim['SM'])

print()
print('Static stability')
print("C_L_alpha: ", sim['C_Lalpha'])
print("C_m_q: ", sim['C_mq'])
print("C_m_alpha: ", sim['C_malpha'])
print("C_l_beta: ", sim['C_lbeta'])
print("C_l_p: ", sim['C_lp'])
print("C_n_beta: ", sim['C_nbeta'])
print("C_y_beta: ", sim['C_ybeta'])
print("C_n_r: ", sim['C_nr'])

print()
print('Dynamic stability')
print("omega_sp: ", sim['w_sp']**(1/2))
print("zeta_sp: ", sim['z_sp']**(1/2))
print("omega_dr: ", sim['w_dr']**(1/2))
print("zeta_dr: ", sim['z_dr']**(1/2))
print("tau_roll: ", sim['t_roll'])

print()
print('wing_cl_dist', wing_cl_dist)

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
