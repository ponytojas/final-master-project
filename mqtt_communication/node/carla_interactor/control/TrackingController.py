from .ControlPose import ControlPose
from .ControlVel import ControlVel
from .control_utils import   n_spline, get_errors, dlqr, get_rc, generate_vel_profile, get_velocity_command

import logging
import numpy as np
import copy
from math import fabs, sqrt, atan2, sin, cos, pi
import os

class TrackingController:
    
    def __init__(self):

        # Sample time
        self.Ts = 0.05

        # Spline coeffs
        self.spline_coeffs = []

        # Velocity profile
        self.vel_profile = []

        # Radius of curvature profile
        self.rc_profile = []

        # Time delay and velocity evolution
        self.n_ret_vel = 4
        self.evol_vel  = [[0] * self.n_ret_vel for _ in range(2)]
        
        # Mechanical parameters
        self.max_steer_angle    = 0.349066
        self.dist_wheels        = 1.8
        self.rc_max             = 20

        # Error and reference
        self.de = 0
        self.oe = 0
        self.lateral_ref = 0

        # LQR Model Matrices
        self.A              = np.zeros((2,2))
        self.B              = np.zeros((2,1))
        self.state_vector   = np.zeros((2,1))
        self.K              = np.zeros((1,2))

        # LQR Weighting Matrices
        self.Q = np.array([[1e0,0],
                           [0,1e-6]])
        self.R = np.eye(1)*1e1

        # Useful flags
        self.spline_generated   = False
        self.stop_requested     = True

        # Poses needed for control
        self.car_pose       = ControlPose()
        self.pose_d         = ControlPose()
        self.future_pose    = ControlPose() 
        self.u_ant          = 0

        # Velocity command
        self.cmd_vel  = ControlVel()

    def update_weights(self, new_Q, new_R):
        self.Q = np.copy(np.array(new_Q))
        self.R = np.copy(np.array(new_R))

    def reset(self):

        # Flags
        self.spline_generated   = False
        self.stop_requested     = True

        # Poses needed for control
        self.car_pose           = ControlPose()
        self.pose_d             = ControlPose()
        self.future_pose        = ControlPose()
        self.u_ant              = 0

        # Velocity command
        self.cmd_vel.v          = 0
        self.cmd_vel.w          = 0

        # Error and reference
        self.de = 0
        self.oe = 0
        self.lateral_ref = 0

        # Spline coeffs
        self.spline_coeffs = []

        # Velocity profile
        self.vel_profile = []

        # Radius of curvature profile
        self.rc_profile = []
        
        # LQR Model Matrices
        self.A              = np.zeros((2,2))
        self.B              = np.zeros((2,1))
        self.state_vector   = np.zeros((2,1))
        self.K              = np.zeros((1,2))

    def trajectory_spline_interpolation(self, control_waypoints):
        self.reset()
        self.spline_coeffs.clear()
        self.spline_coeffs = n_spline(control_waypoints, 0.01)

    def velocity_profile_generation(self, v_road, vmax):
        if len(self.spline_coeffs) > 0:
           self.rc_profile, self.vel_profile = generate_vel_profile(self.spline_coeffs,\
                                                                    v_road, vmax)

    def sol_ecn(self, coef, u_ant):
        '''
            Solver method of equations of degree three in order
            to obtain the parameter 'u'.
        '''

        # Auxiliar variables
        j = 0
        p, p1   = 100, 100
        n = len(coef)-1
        x = copy.copy(u_ant) + 0.1
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
    
    def limit_ang(self, ang) -> float:
        '''
            Method to limit the angle between -pi and pi
        '''
        new_ang = copy.copy(ang)

        while new_ang > pi:
            new_ang -= 2*pi
        while new_ang < -pi:
            new_ang += 2*pi

        return new_ang

    def get_u_setpoint(self, coeffs, car_pose):
        # Get the current polynomial
        coef = []
        act_coeffs = copy.copy(coeffs[car_pose.tramo])
        act_pose   = copy.copy(car_pose)

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
        return self.sol_ecn(coef, car_pose.u)
        
    def get_pose_setpoint(self, coeffs, car_pose):

        # Get the current polynomial
        act_coeffs = copy.copy(coeffs[car_pose.tramo]) if car_pose.tramo < len(coeffs) else copy.copy(coeffs[-1])
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

        if fabs(self.limit_ang(pose_d.o - car_pose.o)) > pi/2:
            self.pose_d.o = self.limit_ang(pose_d.o + pi)
              
        return pose_d

    def locate_pose(self, coeffs, car_pose):
        '''
            Method to locate the current pose in the spline
        '''
        # Copy car
        car = copy.copy(car_pose)

        # Main waypoinyts
        waypoints = np.zeros((len(coeffs), 2))
        for i in range(len(coeffs)):
            waypoints[i,0] = coeffs[i,0]
            waypoints[i,1] = coeffs[i,4]
        
        # Get the closest waypoint
        dist = [sqrt((waypoints[i,0]-car_pose.x)**2 + (waypoints[i,1]-car_pose.y)**2) \
                for i in range(len(coeffs))]
        
        # Get the closest segment
        dist_tramo = [sqrt((dist[i]**2 + dist[i+1])**2)/2 \
                      for i in range(len(coeffs)-1)]
        
        tramo = 0 if len(dist_tramo) == 0 else np.argmin(dist_tramo)

        return tramo, self.get_u_setpoint(coeffs, car)

    def calc_setpoint(self):
        '''
            Method to update de desired pose for a new timestamp
        '''

        # Does the spline exist? If not return
        if len(self.spline_coeffs) == 0:
            return
        
        # Get current 'u'
        u = self.get_u_setpoint(self.spline_coeffs, self.car_pose)

        # Check possible scenarios
        if u >= 0 and u < 1:
            self.u_ant = copy.copy(u)
            self.car_pose.u = u
            return self.get_pose_setpoint(self.spline_coeffs, self.car_pose)
        
        elif u >= 1:
            self.u_ant = 0
            self.car_pose.u = 0
            self.car_pose.tramo += 1
            return self.get_pose_setpoint(self.spline_coeffs, self.car_pose)
        
        else:
            new_tramo, new_u = self.locate_pose(self.spline_coeffs, self.car_pose)
            self.car_pose.u = new_u
            self.car_pose.tramo = new_tramo
            u = self.get_u_setpoint(self.spline_coeffs, self.car_pose)
            self.u_ant = copy.copy(u)
            return self.get_pose_setpoint(self.spline_coeffs, self.car_pose)
        
    def control_move(self):
        
        # Propagation of the pose
        self.future_pose = copy.copy(self.car_pose)
        for i in range(self.n_ret_vel):
            self.future_pose.x = self.car_pose.x + self.evol_vel[0][i] * cos(self.car_pose.o) * self.Ts
            self.future_pose.y = self.car_pose.y + self.evol_vel[0][i] * sin(self.car_pose.o) * self.Ts
            self.future_pose.o = self.car_pose.o + self.evol_vel[1][i] * self.Ts        

        # Get setpoint
        self.pose_d = self.calc_setpoint()
        if self.pose_d == None:
            logging.warning(f"Desired pose is none -> car_pose assigned as desired pose")
            self.pose_d = copy.copy(self.car_pose)

        # Update errors
        self.de = float((self.future_pose.y-self.pose_d.y)*cos(self.pose_d.o)-(self.future_pose.x-self.pose_d.x)*sin(self.pose_d.o))
        self.oe = float(self.limit_ang(self.future_pose.o-self.pose_d.o))
        self.state_vector = np.array([[self.de],
                                      [self.oe]])

        # Velocity profiler -> Linear velocity command
        current_rc, self.cmd_vel.v = get_velocity_command(self.vel_profile, self.rc_profile, self.car_pose)

        # LQR control -> Angular velocity command
        self.A = np.array([[1, self.cmd_vel.v*self.Ts],
                           [0, 1]])
        
        self.B = np.array([[(self.cmd_vel.v*self.Ts**2)/(2*self.dist_wheels)],
                          [(self.cmd_vel.v*self.Ts)/(self.dist_wheels)]])
        
        self.K = dlqr(self.A, self.B, self.Q, self.R)

        if not self.stop_requested:

            self.cmd_vel.w = float(-self.K.dot(self.state_vector)/self.max_steer_angle)

            if self.cmd_vel.w > 1:
                self.cmd_vel.w = 1
            elif self.cmd_vel.w < -1:
                self.cmd_vel.w = -1

        else:
            self.cmd_vel.w = 0
            self.cmd_vel.v = 0

        # Update pose propagation for next step
        for i in range(1, self.n_ret_vel):
            self.evol_vel[0][i-1] = self.evol_vel[0][i]
            self.evol_vel[1][i-1] = self.evol_vel[1][i]        

        self.evol_vel[0][self.n_ret_vel-1] = self.cmd_vel.v
        self.evol_vel[1][self.n_ret_vel-1] = self.cmd_vel.w
