import numpy as np

'''
lift_vals takes wing_span (wing_span) & weight (weight) to output num_airfoils ellipse
coordinates representing individual lift vectors at a distance, y, from adopt.origin.
'''
def get_lift_vals(wing_span, weight, num_airfoils):
    lift_vals = np.zeros((num_airfoils,2))
    a = (8*((weight/2))/0.9)/(np.pi*wing_span)  # a is semi-minor axis length of ellipse
    # y = np.array([0., 0.5, 0.75, 0.9, 1.])*wing_span/2
    y = (np.sin(np.linspace(0., np.pi/2, num_airfoils)))*wing_span/2
    # y = np.linspace(0., wing_span/2, num_airfoils)
    for i in range(num_airfoils):
        lift_vals[i,0] = y[i]
        lift_vals[i,1] = np.sqrt(a**2*(1-(y[i]**2)/(wing_span/2)**2))
    return lift_vals

def get_cl_vals(wing_span, weight, air_density, velocity, chord_root, chord_tip, num_airfoils):
    cl_vals = np.zeros((num_airfoils,2))
    lift_vals = get_lift_vals(wing_span, weight, num_airfoils)
    air_density = 1.225
    for i in range(num_airfoils):
        y = lift_vals[i,0]
        Ly = lift_vals[i,1]
        Cy = chord_root-(chord_root-chord_tip)/(wing_span/2)*y
        cl_vals[i,0] = y
        cl_vals[i,1] = Ly/((1/2)*air_density*velocity**2*Cy)
    return cl_vals
