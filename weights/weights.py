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

wingspan = 11.1
wing_mac = 1.16
wing_taper = 0.45
wing_sweep = 0
W_bwing = 0.5
wing_t = 0.14

tailspan = 3
tail_mac = 1
htail_sweep = 0
htail_taper = 0.45
htail_t = 0.18

vtail_span = 2
vtail_mac = 1
vtail_sweep = 0
vtail_taper = 0.45
vtail_t = 0.12

fuse_width = 1.5    # m
fuse_height = 1.5   # m
fuse_len = 7        # m
tail_len = 3
fus_wet = fuse_len * 2 * np.pi * np.sqrt((fuse_width**2+fuse_height**2)/2)
fus_str_depth = 1/2 * (fuse_width + fuse_height)

S_wing = wingspan * wing_mac
W_ar = wingspan**2 / S_wing
wing_t_over_c = wing_t / wing_mac

S_htail = tailspan * tail_mac
htail_ar = tailspan**2 / S_htail
htail_t_over_c = htail_t / tail_mac

S_vtail = vtail_span * vtail_mac
vtail_ar = vtail_span**2 / S_vtail
vtail_t_over_c = vtail_t / vtail_mac

while True:
    W_wing = 0.036 * (S_wing*unit.mti_area)**0.758 * (battery*unit.mti_weight)**0.0035 * \
            (W_ar/np.cos(wing_sweep)**2)**0.6 * (q*unit.mti_press)**0.006 * \
            wing_taper**0.04 * (100*wing_t_over_c/np.cos(wing_sweep))**(-0.3) * \
            (load_fact_ult*gross*unit.mti_weight)**0.49 * 1/unit.mti_weight

    W_htail = 0.016 * (load_fact_ult*gross*unit.mti_weight)**0.414 * \
            (q*unit.mti_press)**0.168 * (S_htail*unit.mti_area)**0.896 * \
            (100*htail_t_over_c/np.cos(htail_sweep))**(-0.12) * \
            (htail_ar/np.cos(htail_sweep)**2)**0.043 * htail_taper**(-0.02) * \
            1/unit.mti_weight

    W_vtail = 0.073 * (load_fact_ult*gross*unit.mti_weight)**0.376 * \
            (q*unit.mti_press)**0.122 * (S_vtail*unit.mti_area)**0.873 * \
            (100*vtail_t_over_c/np.cos(vtail_sweep))**(-0.49) * \
            (vtail_ar/np.cos(vtail_sweep)**2)**0.357 * vtail_taper**0.039 * \
            1/unit.mti_weight

    W_fuse = 0.052 * (fus_wet*unit.mti_area)**1.086 * \
            (load_fact_ult*gross*unit.mti_weight)**0.177 * \
            (tail_len*unit.mti_len)**(-0.051) * (fuse_len/fus_str_depth)**(-0.072) * \
            (q*unit.mti_press)**0.241 * 1/unit.mti_weight

    empty = empty_struct_fudge * (W_wing + W_htail + W_vtail + W_fuse)
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
weights = empty_struct_fudge * np.array([W_wing, W_htail, W_vtail, W_fuse, 0, 0])
weights += np.array([battery*0.25, 0, 0, payload, 0, battery*0.75])
weights += motor_weight * np.array([8, 0, 0, 0, 2, 0])
weights += misc_weight * np.array([0, 0, 0, 0, 0, 1])

# Everything below assumes no sweep
# Cgs of components from nose
wing_cg = 0.45
htail_cg = 0.9
vtail_cg = 0.9
batt_cg = 0.12

fuse_cg = 0.5 * fuse_len
back_prop_cg = fuse_len + 0.25

wing_cg = wing_cg * fuse_len
htail_cg = htail_cg * fuse_len
vtail_cg = vtail_cg * fuse_len
batt_cg = batt_cg * fuse_len

w1 = W_wing + battery * W_bwing + 8 * motor_weight
w2 = W_htail + 2 * motor_weight
w3 = W_vtail
w4 = W_fuse + payload
w5 = 2 * motor_weight
w6 = (1 - W_bwing) * battery + misc_weight
# ws = np.array([W_wing, W_htail, W_vtail, W_fuse, 0, 0])
# ws += battery_weight * np.array([W_bwing, 0, 0, 0, 0, 1 - W_bwing])
# ws += payload_weight * np.array([0, 0, 0, 1, 0, 0])
# ws += motor_weight * np.array([8, 0, 0, 0, 2, 0])
# ws += misc_weight * np.array([0, 0, 0, 0, 0, 1])

# cgs = fuse_len * np.array([wing_cg, htail_cg, vtail_cg, fuse_cg, back_prop_cg, batt_cg])
# print(cgs[0])
# weighted_cgs = cgs[0]*ws[0]+cgs[1]*ws[1]+cgs[2]*ws[2]+cgs[3]*ws[3]+cgs[4]*ws[4]+cgs[5]*ws[5]
# cg = weighted_cg / ot.sum(weights)
wx = w1*wing_cg + w2*htail_cg + w3*vtail_cg + w4*fuse_cg + w5*back_prop_cg + w6*batt_cg
cg = wx / (w1+w2+w3+w4+w5+w6)

print(wing_cg)
print(htail_cg)
print(vtail_cg)
print(fuse_cg)
print(back_prop_cg)
print(batt_cg)

print(w1/g)
print(w2/g)
print(w3/g)
print(w4/g)
print(w5/g)
print(w6/g)

# wing_cg = 3.2
# htail_cg = 6.25
# vtail_cg = 6.25
# fuse_cg = 3.5
# back_prop_cg = 7.25
# batt_cg = 0.75
# 
# cg_indiv = np.array([wing_cg, htail_cg, vtail_cg, fuse_cg, back_prop_cg, batt_cg])
# 
# cg = (weights @ cg_indiv) / sum(weights)

#print(gross)
#print(sum(weights))

C_l_alpha = 2*np.pi

C_L_alpha_w = C_l_alpha/(1+C_l_alpha/(np.pi*W_ar))
C_L_alphat_h = C_l_alpha/(1+C_l_alpha/(np.pi*htail_ar))
C_L_alphat_v = C_l_alpha/(1+C_l_alpha/(np.pi*vtail_ar))

deps_dalpha = 0.45
eta_h = 0.9
dsigma_dbeta = (S_vtail / S_wing) * 3.06 / (1 + np.cos(wing_sweep)) + 0.009 * W_ar - 0.276
eta_v = 0.9

C_L_alpha_h = C_L_alphat_h * (1 - deps_dalpha) * eta_h
C_L_alpha_v = C_L_alphat_h * (1 + dsigma_dbeta) * eta_v

W_ac = wing_cg - wing_mac / 4
htail_ac = htail_cg - tail_mac / 4
vtail_ac = vtail_cg - vtail_mac / 4

x_np = (C_L_alpha_w * S_wing * W_ac + C_L_alpha_h * S_htail * htail_ac) / \
        (C_L_alpha_w * S_wing + C_L_alpha_h * S_htail)

SM = (x_np - cg) / wing_mac * 100

L_h = htail_ac - W_ac
c_h = L_h * S_htail / wing_mac / S_wing

L_v = vtail_ac - W_ac
c_v = L_v * S_vtail / wingspan / S_wing

htail_ratio = S_htail / S_wing
vtail_ratio = S_vtail / S_wing

C_Lalpha = C_L_alpha_w + C_L_alpha_h * htail_ratio
C_mq = -2 * eta_h * C_L_alphat_h * htail_ratio * ((htail_ac - cg) / wing_mac)**2
C_malpha = C_L_alpha_w * ((cg-W_ac)/wing_mac) - C_L_alpha_h * htail_ratio * ((htail_ac-cg)/wing_mac)

dihedral = 2 / 180 * np.pi

C_l_beta = -0.66 * dihedral
C_l_p = -C_Lalpha/12*(1+3*wing_taper)/(1+wing_taper)
C_nbeta = C_L_alpha_v * vtail_ratio * (vtail_ac - cg) / wingspan
C_ybeta = -C_L_alpha_v * vtail_ratio
C_nr = -C_L_alphat_v * eta_v * vtail_ratio * ((vtail_ac-cg)/wingspan)**2

Rx = 0.25
Ry = 0.38
Rz = 0.39

Ixx = (wingspan*unit.mti_len)**2 * (gross*unit.mti_weight) * Rx**2 / unit.g / 4 / unit.mti_inertia
Iyy = (fuse_len*unit.mti_len)**2 * (gross*unit.mti_weight) * Ry**2 / unit.g / 4 / unit.mti_inertia
Izz = ((fuse_len+wingspan)/2*unit.mti_len)**2 * (gross*unit.mti_weight) * Rz**2 / unit.g / 4 / unit.mti_inertia

C_L_q = 0.01    # This is a guess. I have no equation
C_yr = 0.25     # Also a guess lol

m = gross/g
V_0 = cruise_vel
Z_w = -q*S_wing/m*C_Lalpha/V_0
M_q = q*wing_mac**2*S_wing/2/V_0/Iyy*C_mq
M_w = q*wing_mac*S_wing/Iyy*C_malpha/V_0
Z_q = -q*wing_mac*S_wing/2/m/V_0*C_L_q
Y_v = q*S_wing/m*C_ybeta/V_0
N_r = q*S_wing*wingspan**2/2/Izz/V_0*C_nr
N_v = q*S_wing*wingspan/Izz*C_nbeta/V_0
Y_r = q*S_wing*wingspan/2/m/V_0*C_yr
L_p = q*S_wing*wingspan**2/2/Ixx/V_0*C_l_p

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
print("CG location: ", cg)
print("Neutral point: ", x_np)
print("SM: ", SM)
print("Horizontal tail volume coeff: ", c_h)
print("Vertical tail volume coeff: ", c_v)
print("Battery weight: ", battery/g)
print("Payload weight: ", payload/g)
print("Wing weight: ", W_wing*empty_struct_fudge/g)
print("Htail weight: ", W_htail*empty_struct_fudge/g)
print("Vtail weight: ", W_vtail*empty_struct_fudge/g)
print("Fuselage weight: ", W_fuse*empty_struct_fudge/g)
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
print("Wing weight: ", W_wing*empty_struct_fudge/gross)
print("Htail weight: ", W_htail*empty_struct_fudge/gross)
print("Vtail weight: ", W_vtail*empty_struct_fudge/gross)
print("Fuselage weight: ", W_fuse*empty_struct_fudge/gross)
print("Motor weight: ", num_motors*motor_weight/gross)
print("Misc. weight: ", misc_weight/gross)
