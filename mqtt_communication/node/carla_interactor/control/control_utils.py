from .ControlPose import ControlPose
from math import cos, sin, pi, sqrt, fabs, atan2
from copy import copy

import numpy as np

# Spline
def n_spline(waypoints_path, nu=0.001, lateral_offsets=[]):
  '''
  Input:
      · waypoints: (X,Y,O).
      · nu: smoothing factor
      · lateral_offsets: list of lateral offsets for each waypoint. 
      If it is None, no lateral offsets are applied (offset = 0).

  Output:
      · coeffs: List of tuples defining the polynomials.

  '''
  waypoints = np.array(waypoints_path[:])
  m = len(waypoints)
  coeffs = np.zeros((m-1, 8))
  offsets = lateral_offsets[:]

  lam = np.zeros(m)
  deta = np.zeros(m)
  D = np.zeros(m)

  if len(offsets) != 0:
    for offset, waypoint in zip(lateral_offsets, waypoints):
      waypoint.x = waypoint.x + offset*sin(waypoint.o)
      waypoint.y = waypoint.y - offset*cos(waypoint.o)

  # X(u) #
  lam[0] = 0 
  deta[0] = nu*cos(waypoints[0].o)

  for i in range(1, m-1):
    lam[i] = (1 / (4 - lam[i-1]))
    deta[i] = (3 * (waypoints[i+1].x - waypoints[i-1].x) - deta[i-1]) * lam[i]
 
  deta[-1] = nu*cos(waypoints[-1].o)
  D[-1] = deta[-1]

  for i in range(m-2, 0, -1):
    D[i] = deta[i] - lam[i] * D[i+1]

  for i in range(m-1):
    coeffs[i,0] = waypoints[i].x
    coeffs[i,1] = D[i]
    coeffs[i,2] = 3 * (waypoints[i+1].x - waypoints[i].x) - 2 * D[i] - D[i+1]
    coeffs[i,3] = 2 * (waypoints[i].x - waypoints[i+1].x) + D[i] + D[i+1]
  
  # Y(u) #
  lam[0] = 0
  deta[0] = nu*sin(waypoints[0].o)

  for i in range(1, m-1):
    lam[i] = 1 / (4 - lam[i-1])
    deta[i] = ((3 * (waypoints[i+1].y - waypoints[i-1].y) - deta[i-1]) * lam[i])
  deta[-1] = nu * sin(waypoints[-1].o)

  D[m-1] = deta[m-1]

  for i in range(m-2, 0, -1):
    D[i] = deta[i] - lam[i] * D[i+1]

  for i in range(m - 1):
    coeffs[i,4] = waypoints[i].y
    coeffs[i,5] = D[i]
    coeffs[i,6] = 3 * (waypoints[i+1].y - waypoints[i].y) - 2 * D[i] - D[i+1]
    coeffs[i,7] = 2 * (waypoints[i].y - waypoints[i+1].y) + D[i] + D[i+1]

  return coeffs

# Radius of curvature profile
def generate_rc_profile(coefs, rc_max = 100) -> list:
    '''
    Input:
        · coefs: List of tuples defining the polynomials.
        · v_road: Road speed.
        · vmax: Maximum speed.

    Output:
        · vel_profile: List of tuples defining the velocity profile.
        · rc_profile: List defining the curvature radius profile.
    '''
  
    # Variables
    rc_profile = []

    # Curvature radius
    rc, rc_min = 1, 1

    # Segment
    tramos = len(coefs)

    # Curvature profile calculation
    for i in range(tramos):

        rc_sum = 0
        num_it = 0
        u = 0.01

        while u <= 1:
            der1_x = (3*coefs[i][3]*u**2) + (2*coefs[i][2]*u) + coefs[i][1]
            der1_y = (3*coefs[i][7]*u**2) + (2*coefs[i][6]*u) + coefs[i][5]
            der1   = der1_y/der1_x
            der2_x = (6*coefs[i][3]*u) + (2*coefs[i][2])
            der2_y = (6*coefs[i][7]*u) + (2*coefs[i][6])
            der2   = (der2_y*der1_x - der2_x*der1_y)/(der1_x**3)
            der2   = fabs(der2)

            if der2 < 0.001: rc = rc_max
            else:
                aux = 1+der1**2
                rc  = sqrt(aux**3)
                rc  = rc/der2
            
            if rc < rc_min: rc_min = rc
            elif rc > rc_max: rc = rc_max

            num_it += 1
            u += 0.01
            rc_sum += rc

        rc_sum = rc_sum/num_it
        rc_profile.append(rc_sum)

    return rc_profile

# Get current rc based on rc profile
def get_rc_from_profile(rc_profile, tramo) -> float:
    return rc_profile[tramo] if tramo < len(rc_profile) else rc_profile[-1]

# Get current rc based on desired pose
def get_rc(vehicle_pose, coeffs):

    # Max y min
    rc_max = 100
    rc_min = 1

    # Índices
    i = vehicle_pose.tramo
    u = vehicle_pose.u

    # print(f"Tramo: {i}, u: {u}")
    if 0 <= u <= 1:
        der1_x = (3*coeffs[i][3]*u**2) + (2*coeffs[i][2]*u) + coeffs[i][1]
        der1_x = der1_x if der1_x != 0 else 0.0001        
        der1_y = (3*coeffs[i][7]*u**2) + (2*coeffs[i][6]*u) + coeffs[i][5]
        der1   = der1_y/der1_x
        der2_x = (6*coeffs[i][3]*u) + (2*coeffs[i][2])
        der2_y = (6*coeffs[i][7]*u) + (2*coeffs[i][6])
        der2   = (der2_y*der1_x - der2_x*der1_y)/(der1_x**3)
        der2   = fabs(der2)

        if der2 < 0.001: rc = rc_max
        else:
            aux = 1+der1**2
            rc  = sqrt(aux**3)
            rc  = rc/der2
        
        if rc < rc_min: rc_min = rc
        elif rc > rc_max: rc = rc_max
    else:
        rc = -1

    return rc

# Solver method
def sol_ecn(coef, u_ant):
    '''
        Solver method of equations of degree three in order
        to obtain the parameter 'u'.
    '''

    # Auxiliar variables
    j = 0
    p, p1   = 100, 100
    n = len(coef)-1
    x = copy(u_ant) + 0.1
    emax = 0.0001

    while fabs(p) > emax and j<100:
        p = coef[n]*x+coef[n-1]
        p1 = coef[n]

        for i in range(n-2, -1, -1):
            p1  = p+p1*x
            p   = coef[i]+p*x
    
        if fabs(p) > emax:
            x -= p/p1

        j += 1
    
    if x<0:
        p = 1
        j = 0
        x += 0.2

        while fabs(p) > emax and j<100:
            p = coef[n]*x+coef[n-1]
            p1 = coef[n]

            for i in range(n-2, -1, -1):
                p1  = p+p1*x
                p   = coef[i]+p*x
        
            if fabs(p) > emax:
                x -= p/p1

            j += 1

    return x

# Limit angle
def limit_ang(ang) -> float:
        '''
            Method to limit the angle between -pi and pi
        '''
        new_ang = copy(ang)

        while new_ang > pi:
            new_ang -= 2*pi
        while new_ang < -pi:
            new_ang += 2*pi

        return new_ang

# Get the position of the vehicle in the spline
def calc_setpoint(spline_coeffs, car_pose) -> ControlPose:
    '''
        Method to update de desired pose for a new timestamp
    '''

    # Check the coeffs
    if not spline_coeffs:
        return None

    # Get the current polynomial
    coef = []
    act_coeffs = copy(spline_coeffs[car_pose.tramo]) \
                    if car_pose.tramo < len(spline_coeffs)\
                    else copy(spline_coeffs[-1])
    act_pose = copy(car_pose)

    coef.append(act_coeffs[0]*act_coeffs[1] -
                act_pose.x*act_coeffs[1] +
                act_coeffs[4]*act_coeffs[5] -
                act_pose.y*act_coeffs[5])
    
    coef.append(act_coeffs[1]**2 -
                2*act_pose.x*act_coeffs[2] +
                2*act_coeffs[0]*act_coeffs[2] +
                act_coeffs[5]**2 -
                2*act_pose.y*act_coeffs[6] +
                2*act_coeffs[4]*act_coeffs[6])
    
    coef.append((3*act_coeffs[2]*act_coeffs[1])-
                (3*act_pose.x*act_coeffs[3])+(3*act_coeffs[0]*act_coeffs[3])+
                (3*act_coeffs[5]*act_coeffs[6])-(3*act_pose.y*act_coeffs[7])+(
                    3*act_coeffs[4]*act_coeffs[7]))
    
    coef.append((2*act_coeffs[2]*act_coeffs[2])+
                (4*act_coeffs[1]*act_coeffs[3])+
                (2*act_coeffs[6]*act_coeffs[6])+(
                    4*act_coeffs[5]*act_coeffs[7]))
    
    coef.append((5*act_coeffs[2]*act_coeffs[3])+
                (5*act_coeffs[6]*act_coeffs[7]))
    
    coef.append((3*act_coeffs[3]*act_coeffs[3])+
                (3*act_coeffs[7]*act_coeffs[7]))

    # Get new u
    car_pose.u = sol_ecn(coef, car_pose.u)

    if car_pose.tramo == 0 and car_pose.u < 0:
        car_pose.u = 0   

    # Get desired pose
    pose_d = ControlPose()
    pose_d.x =     act_coeffs[0] + act_coeffs[1]*car_pose.u + \
                        act_coeffs[2]*car_pose.u**2 + \
                        act_coeffs[3]*car_pose.u**3
    
    pose_d.y =    act_coeffs[4] + act_coeffs[5]*car_pose.u + \
                        act_coeffs[6]*car_pose.u**2 + \
                        act_coeffs[7]*car_pose.u**3
    
    pose_d.o = atan2(act_coeffs[5] + 2*act_coeffs[6]*car_pose.u + \
                        3*act_coeffs[7]*car_pose.u**2, \
                        act_coeffs[1] + 2*act_coeffs[2]*car_pose.u + \
                        3*act_coeffs[3]*car_pose.u**2)    

    if fabs(limit_ang(pose_d.o - car_pose.o)) > pi/2:
        pose_d.o = limit_ang(pose_d.o + pi)

    # Check if we have to change the segment
    if car_pose.u > 1:
        car_pose.u = 0
        car_pose.tramo += 1
        calc_setpoint(spline_coeffs, car_pose)

    return pose_d

# Velocity profile
def generate_vel_profile(coefs, v_road, vmax, rc_max=20):
    '''
    Input:
        · coefs: List of tuples defining the polynomials.
        · v_road: Road speed.
        · vmax: Maximum speed.

    Output:
        · vel_profile: List of tuples defining the velocity profile.
        · rc_profile: List defining the curvature radius profile.
    '''
  
    # Variables
    rc_profile = []
    vel_profile = []

    # Curvature radius
    rc_min = 1
    rc = np.zeros(101)  # Since u goes from 0.01 to 1 in steps of 0.01

    # Segment
    tramos = len(coefs)

    # Pre-calculate u values
    u = np.linspace(0.01, 1, 100)

    # Curvature profile calculation
    for poly in coefs:
        # Compute the derivatives for x and y over all u at once
        der1_x = (3 * poly[3] * u**2) + (2 * poly[2] * u) + poly[1]
        der1_y = (3 * poly[7] * u**2) + (2 * poly[6] * u) + poly[5]
        der2_x = (6 * poly[3] * u) + (2 * poly[2])
        der2_y = (6 * poly[7] * u) + (2 * poly[6])

        # Compute curvature for each u
        der1 = der1_y / der1_x
        der2 = (der2_y * der1_x - der2_x * der1_y) / (der1_x**3)
        der2 = np.abs(der2)

        rc = np.where(der2 < 0.001, rc_max, np.sqrt((1 + der1**2)**3) / der2)
        rc = np.clip(rc, rc_min, rc_max)

        # Compute mean curvature for this segment
        rc_mean = np.mean(rc)
        rc_profile.append(rc_mean)

    # Velocity profile calculation
    for i in range(tramos):
        vel = (min(v_road[i], vmax) * rc_profile[i]) / rc_max
        vel_profile.append(vel)

    return rc_profile, vel_profile

def get_velocity_command(vel_profile, rc_profile, car_pose):
    '''
    Input:
        · vel_profile: List of tuples defining the velocity profile.
        · car_pose: Rbs2_ControlPose -> We need current u value and segment from spline
    Output:
        · vel_command: Velocity command
        · rc : Curvature radius
    '''

    tramo = car_pose.tramo - 1

    if vel_profile == []:
        return 100.0, 0.0
    else:
        if car_pose.u > 0.5:
            vel_command = (
                vel_profile[tramo]
                +(car_pose.u-0.5)*(vel_profile[tramo+1]-vel_profile[tramo])
            )
        else:
            vel_command = (
                vel_profile[tramo-1]
                +(car_pose.u+0.5)*(vel_profile[tramo]-vel_profile[tramo-1])
            )
        
        return rc_profile[tramo], vel_command

# Get lateral and orientation errors
def get_errors(car_pose, pose_d):
    '''
        Method to get the lateral and orientation errors
    '''

    # Update errors
    de = float((car_pose.y-pose_d.y)*cos(pose_d.o)-(car_pose.x-pose_d.x)*sin(pose_d.o))
    oe = float(limit_ang(car_pose.o-pose_d.o))
    
    # if de > 1:
    #     print(f"ERROR: {de}")
        
    #     print(f"car_pose x: {car_pose.x}")
    #     print(f"car_pose y: {car_pose.y}")
    #     print(f"car_pose u: {car_pose.u}")
    #     print(f"car_pose tramo: {car_pose.tramo}")

    #     print(f"pose_d x: {pose_d.x}")
    #     print(f"pose_d y: {pose_d.y}")

    return de, oe

# LQR
def dlqr(A, B, Q, R, n_it=30):

    # 1. Calculate de Ricatti equation solution
    
    # 1.1 We give an initial value for P_inf
    P_inf = 3.5*np.eye(Q.shape[1])
    K_inf = 3.5*np.ones(R.shape[1])
    # 1.2 We iterate n_it times until we get an acceptable value of P_inf
    for _ in range(n_it):
        K_inf = -np.linalg.inv(np.transpose(B)@P_inf@B + R)@np.transpose(B)@P_inf@A
        P_inf = np.transpose(A)@P_inf@A + np.transpose(A)@P_inf@B@K_inf + Q

    return K_inf   