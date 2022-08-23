import numpy as np
import unitconv as unit

g = 9.81            # m/s^2
payload = 400*g     # N
battery = 300*g     # N

gross = 1300*g      # N
cruise_vel = 40     # m/s
air_dens = 1.225    # kg/m^3

load_fact_ult = 4.5
empty_struct_fudge = 0.9
misc_weight_fudge = 0.25

q = 1/2*air_dens*cruise_vel**2

wing_span = 11.1
wing_mean_aerodynamic_chord = 1.16
wing_taper = 0.45
wing_sweep = 0
wing_battery_weight_ratio = 0.5
wing_thickness = 0.14

tailspan = 3
tail_mean_aerodynamic_chord = 1
horizontal_stabilizer_sweep = 0
horizontal_stabilizer_taper = 0.45
horizontal_stabilizer_thickness = 0.18

vertical_stabilizer_span = 2
vertical_stabilizer_mean_aerodynamic_chord = 1
vertical_stabilizer_sweep = 0
vertical_stabilizer_taper = 0.45
vertical_stabilizer_thickness = 0.12

fuselage_width = 1.5    # m
fuselage_height = 1.5   # m
fuselage_len = 7        # m
tail_len = 3
fuselage_wetted = fuselage_len * 2 * np.pi * np.sqrt((fuselage_width**2+fuselage_height**2)/2)
fuselage_depth = 1/2 * (fuselage_width + fuselage_height)

wing_area = wing_span * wing_mean_aerodynamic_chord
wing_aspect_ratio = wing_span**2 / wing_area
wing_thickness_to_chord_ratio = wing_thickness / wing_mean_aerodynamic_chord

horizontal_stabilizer_area = tailspan * tail_mean_aerodynamic_chord
horizontal_stabilizer_aspect_ratio = tailspan**2 / horizontal_stabilizer_area
horizontal_stabilizer_thickness_to_chord_ratio = horizontal_stabilizer_thickness / tail_mean_aerodynamic_chord

vertical_stabilizer_area = vertical_stabilizer_span * vertical_stabilizer_mean_aerodynamic_chord
vertical_stabilizer_aspect_ratio = vertical_stabilizer_span**2 / vertical_stabilizer_area
vertical_stabilizer_thickness_to_chord_ratio = vertical_stabilizer_thickness / vertical_stabilizer_mean_aerodynamic_chord

while True:
    wing_weight = 0.036 * (wing_area*unit.mti_area)**0.758 * (battery*unit.mti_weight)**0.0035 * \
            (wing_aspect_ratio/np.cos(wing_sweep)**2)**0.6 * (q*unit.mti_press)**0.006 * \
            wing_taper**0.04 * (100*wing_thickness_to_chord_ratio/np.cos(wing_sweep))**(-0.3) * \
            (load_fact_ult*gross*unit.mti_weight)**0.49 * 1/unit.mti_weight

    horitontal_stabilizer_weight = 0.016 * (load_fact_ult*gross*unit.mti_weight)**0.414 * \
            (q*unit.mti_press)**0.168 * (horizontal_stabilizer_area*unit.mti_area)**0.896 * \
            (100*horizontal_stabilizer_thickness_to_chord_ratio/np.cos(horizontal_stabilizer_sweep))**(-0.12) * \
            (horizontal_stabilizer_aspect_ratio/np.cos(horizontal_stabilizer_sweep)**2)**0.043 * horizontal_stabilizer_taper**(-0.02) * \
            1/unit.mti_weight

    vertical_stabilizer_weight = 0.073 * (load_fact_ult*gross*unit.mti_weight)**0.376 * \
            (q*unit.mti_press)**0.122 * (vertical_stabilizer_area*unit.mti_area)**0.873 * \
            (100*vertical_stabilizer_thickness_to_chord_ratio/np.cos(vertical_stabilizer_sweep))**(-0.49) * \
            (vertical_stabilizer_aspect_ratio/np.cos(vertical_stabilizer_sweep)**2)**0.357 * vertical_stabilizer_taper**0.039 * \
            1/unit.mti_weight

    fuselage_weight = 0.052 * (fuselage_wetted*unit.mti_area)**1.086 * \
            (load_fact_ult*gross*unit.mti_weight)**0.177 * \
            (tail_len*unit.mti_len)**(-0.051) * (fuselage_len/fuselage_depth)**(-0.072) * \
            (q*unit.mti_press)**0.241 * 1/unit.mti_weight

    empty = empty_struct_fudge * (wing_weight + horitontal_stabilizer_weight + vertical_stabilizer_weight + fuselage_weight)
    new_gross = (1 + misc_weight_fudge) * (payload + empty + battery)

    if abs(new_gross - gross) < 0.1:
        gross = new_gross
        break

    gross = new_gross

misc_weight = misc_weight_fudge*(payload+empty+battery)
num_motors = 10
motor_weight = 0.75*misc_weight / num_motors
misc_weight -= motor_weight*num_motors

# List of weights
weights = empty_struct_fudge * np.array([wing_weight, horitontal_stabilizer_weight, vertical_stabilizer_weight, fuselage_weight, 0, 0])
weights += np.array([battery*0.25, 0, 0, payload, 0, battery*0.75])
weights += motor_weight * np.array([8, 0, 0, 0, 2, 0])
weights += misc_weight * np.array([0, 0, 0, 0, 0, 1])

# Everything below assumes no sweep
# Cgs of components from adopt.nose
wing_center_of_mass = 0.45
horizontal_stabilizer_center_of_mass = 0.9
vertical_stabilizer_center_of_mass = 0.9
battery_center_of_mass = 0.12

fuselage_center_of_mass = 0.5 * fuselage_len
back_prop_center_of_mass = fuselage_len + 0.25

wing_center_of_mass = wing_center_of_mass * fuselage_len
horizontal_stabilizer_center_of_mass = horizontal_stabilizer_center_of_mass * fuselage_len
vertical_stabilizer_center_of_mass = vertical_stabilizer_center_of_mass * fuselage_len
battery_center_of_mass = battery_center_of_mass * fuselage_len

w1 = wing_weight + battery * wing_battery_weight_ratio + 8 * motor_weight
w2 = horitontal_stabilizer_weight + 2 * motor_weight
w3 = vertical_stabilizer_weight
w4 = fuselage_weight + payload
w5 = 2 * motor_weight
w6 = (1 - wing_battery_weight_ratio) * battery + misc_weight
# ws = np.array([wing_weight, horitontal_stabilizer_weight, vertical_stabilizer_weight, fuselage_weight, 0, 0])
# ws += battery_weight * np.array([wing_battery_weight_ratio, 0, 0, 0, 0, 1 - wing_battery_weight_ratio])
# ws += payload_weight * np.array([0, 0, 0, 1, 0, 0])
# ws += motor_weight * np.array([8, 0, 0, 0, 2, 0])
# ws += misc_weight * np.array([0, 0, 0, 0, 0, 1])

# center_of_masss = fuselage_len * np.array([wing_center_of_mass, horizontal_stabilizer_center_of_mass, vertical_stabilizer_center_of_mass, fuselage_center_of_mass, back_prop_center_of_mass, battery_center_of_mass])
# print(center_of_masss[0])
# weighted_center_of_masss = center_of_masss[0]*ws[0]+center_of_masss[1]*ws[1]+center_of_masss[2]*ws[2]+center_of_masss[3]*ws[3]+center_of_masss[4]*ws[4]+center_of_masss[5]*ws[5]
# center_of_mass = weighted_center_of_mass / ot.sum(weights)
wx = w1*wing_center_of_mass + w2*horizontal_stabilizer_center_of_mass + w3*vertical_stabilizer_center_of_mass + w4*fuselage_center_of_mass + w5*back_prop_center_of_mass + w6*battery_center_of_mass
center_of_mass = wx / (w1+w2+w3+w4+w5+w6)

print(wing_center_of_mass)
print(horizontal_stabilizer_center_of_mass)
print(vertical_stabilizer_center_of_mass)
print(fuselage_center_of_mass)
print(back_prop_center_of_mass)
print(battery_center_of_mass)

print(w1/g)
print(w2/g)
print(w3/g)
print(w4/g)
print(w5/g)
print(w6/g)

# wing_center_of_mass = 3.2
# horizontal_stabilizer_center_of_mass = 6.25
# vertical_stabilizer_center_of_mass = 6.25
# fuselage_center_of_mass = 3.5
# back_prop_center_of_mass = 7.25
# battery_center_of_mass = 0.75
# 
# center_of_mass_indiv = np.array([wing_center_of_mass, horizontal_stabilizer_center_of_mass, vertical_stabilizer_center_of_mass, fuselage_center_of_mass, back_prop_center_of_mass, battery_center_of_mass])
# 
# center_of_mass = (weights @ center_of_mass_indiv) / sum(weights)

#print(gross)
#print(sum(weights))

C_l_alpha = 2*np.pi

C_L_alpha_w = C_l_alpha/(1+C_l_alpha/(np.pi*wing_aspect_ratio))
C_L_alphat_h = C_l_alpha/(1+C_l_alpha/(np.pi*horizontal_stabilizer_aspect_ratio))
C_L_alphat_v = C_l_alpha/(1+C_l_alpha/(np.pi*vertical_stabilizer_aspect_ratio))

deps_dalpha = 0.45
eta_h = 0.9
dsigma_dbeta = (vertical_stabilizer_area / wing_area) * 3.06 / (1 + np.cos(wing_sweep)) + 0.009 * wing_aspect_ratio - 0.276
eta_v = 0.9

C_L_alpha_h = C_L_alphat_h * (1 - deps_dalpha) * eta_h
C_L_alpha_v = C_L_alphat_h * (1 + dsigma_dbeta) * eta_v

W_ac = wing_center_of_mass - wing_mean_aerodynamic_chord / 4
horizontal_stabilizer_ac = horizontal_stabilizer_center_of_mass - tail_mean_aerodynamic_chord / 4
vertical_stabilizer_ac = vertical_stabilizer_center_of_mass - vertical_stabilizer_mean_aerodynamic_chord / 4

x_np = (C_L_alpha_w * wing_area * W_ac + C_L_alpha_h * horizontal_stabilizer_area * horizontal_stabilizer_ac) / \
        (C_L_alpha_w * wing_area + C_L_alpha_h * horizontal_stabilizer_area)

SM = (x_np - center_of_mass) / wing_mean_aerodynamic_chord * 100

L_h = horizontal_stabilizer_ac - W_ac
c_h = L_h * horizontal_stabilizer_area / wing_mean_aerodynamic_chord / wing_area

L_v = vertical_stabilizer_ac - W_ac
c_v = L_v * vertical_stabilizer_area / wing_span / wing_area

horizontal_stabilizer_ratio = horizontal_stabilizer_area / wing_area
vertical_stabilizer_ratio = vertical_stabilizer_area / wing_area

C_Lalpha = C_L_alpha_w + C_L_alpha_h * horizontal_stabilizer_ratio
C_mq = -2 * eta_h * C_L_alphat_h * horizontal_stabilizer_ratio * ((horizontal_stabilizer_ac - center_of_mass) / wing_mean_aerodynamic_chord)**2
C_malpha = C_L_alpha_w * ((center_of_mass-W_ac)/wing_mean_aerodynamic_chord) - C_L_alpha_h * horizontal_stabilizer_ratio * ((horizontal_stabilizer_ac-center_of_mass)/wing_mean_aerodynamic_chord)

wing_dihedral = 2 / 180 * np.pi

C_l_beta = -0.66 * wing_dihedral
C_l_p = -C_Lalpha/12*(1+3*wing_taper)/(1+wing_taper)
C_nbeta = C_L_alpha_v * vertical_stabilizer_ratio * (vertical_stabilizer_ac - center_of_mass) / wing_span
C_ybeta = -C_L_alpha_v * vertical_stabilizer_ratio
C_nr = -C_L_alphat_v * eta_v * vertical_stabilizer_ratio * ((vertical_stabilizer_ac-center_of_mass)/wing_span)**2

Rx = 0.25
Ry = 0.38
Rz = 0.39

Ixx = (wing_span*unit.mti_len)**2 * (gross*unit.mti_weight) * Rx**2 / unit.g / 4 / unit.mti_inertia
Iyy = (fuselage_len*unit.mti_len)**2 * (gross*unit.mti_weight) * Ry**2 / unit.g / 4 / unit.mti_inertia
Izz = ((fuselage_len+wing_span)/2*unit.mti_len)**2 * (gross*unit.mti_weight) * Rz**2 / unit.g / 4 / unit.mti_inertia

C_L_q = 0.01    # This is a guess. I have no equation
C_yr = 0.25     # Also a guess lol

m = gross/g
V_0 = cruise_vel
Z_w = -q*wing_area/m*C_Lalpha/V_0
M_q = q*wing_mean_aerodynamic_chord**2*wing_area/2/V_0/Iyy*C_mq
M_w = q*wing_mean_aerodynamic_chord*wing_area/Iyy*C_malpha/V_0
Z_q = -q*wing_mean_aerodynamic_chord*wing_area/2/m/V_0*C_L_q
Y_v = q*wing_area/m*C_ybeta/V_0
N_r = q*wing_area*wing_span**2/2/Izz/V_0*C_nr
N_v = q*wing_area*wing_span/Izz*C_nbeta/V_0
Y_r = q*wing_area*wing_span/2/m/V_0*C_yr
L_p = q*wing_area*wing_span**2/2/Ixx/V_0*C_l_p

omega_sp = np.sqrt(Z_w*M_q - M_w*(V_0+Z_q))
zeta_sp = -(Z_w+M_q)/2/omega_sp
omega_dr = np.sqrt(Y_v*N_r + N_v*(V_0-Y_r))
zeta_dr = -(Y_v+N_r)/2/omega_dr
tau_r = -1/L_p

print(Z_w)
print(M_q)
print(M_w)
print(V_0)
print(Z_q)
print(Z_w*M_q - M_w*(V_0+Z_q))

print("w_sp: ", omega_sp)
print("zeta_sp: ", zeta_sp)
print("w_dr: ", omega_dr)
print("zeta_dr: ", zeta_dr)
print("t_r: ", tau_r)

print()
print("Gross weight: ", gross/g)
print("CG location: ", center_of_mass)
print("Neutral point: ", x_np)
print("SM: ", SM)
print("Horizontal tail volume coeff: ", c_h)
print("Vertical tail volume coeff: ", c_v)
print("Battery weight: ", battery/g)
print("Payload weight: ", payload/g)
print("Wing weight: ", wing_weight*empty_struct_fudge/g)
print("Htail weight: ", horitontal_stabilizer_weight*empty_struct_fudge/g)
print("Vtail weight: ", vertical_stabilizer_weight*empty_struct_fudge/g)
print("Fuselage weight: ", fuselage_weight*empty_struct_fudge/g)
print("Motor weight: ", num_motors*motor_weight/g)
print("Misc. weight: ", misc_weight/g)

print()
print("C_L_alpha: ", C_Lalpha)
print("C_m_q: ", C_mq)
print("C_m_alpha: ", C_malpha)
print("C_l_beta: ", C_l_beta)
print("C_l_p: ", C_l_p)
print("C_n_beta: ", C_nbeta)
print("C_y_beta: ", C_ybeta)
print("C_n_r: ", C_nr)

print()
print("Battery weight: ", battery/gross)
print("Payload weight: ", payload/gross)
print("Wing weight: ", wing_weight*empty_struct_fudge/gross)
print("Htail weight: ", horitontal_stabilizer_weight*empty_struct_fudge/gross)
print("Vtail weight: ", vertical_stabilizer_weight*empty_struct_fudge/gross)
print("Fuselage weight: ", fuselage_weight*empty_struct_fudge/gross)
print("Motor weight: ", num_motors*motor_weight/gross)
print("Misc. weight: ", misc_weight/gross)
