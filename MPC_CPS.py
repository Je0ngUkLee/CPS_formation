#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@MPC for Quadrotor

@file MPC_CPS.py
@author Jeong-Uk Lee (ju.lee@sch.ac.kr)
@version 0.1
@date 2023-08-10 16:04

"""

import casadi as ca
import numpy  as np

import signal
import rospy
import sys
import math
import timeit

from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

from multiprocessing import Queue

from tf.transformations import euler_from_quaternion, quaternion_from_euler


'''
Ctrl+C : program exit
'''

def signal_handler(signal, frame):
  print('\nYou pressed Ctrl+C!')
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def shift(x_n, u):  
  x_n = np.delete(x_n, 0, axis = 1)
  col_add_list_x = x_n[:, -1]
  x_n = np.c_[x_n, col_add_list_x]
  
  u = np.delete(u, 0, axis = 1)
  col_add_list_u = u[:, -1]
  u = np.c_[u, col_add_list_u]
  return x_n, u


'''
MPC class
'''

class MPC():
  def __init__(self):
    
    # UAV Parameters
    # self.mass       = 1.52
    self.mass       = 2.85
    self.arm_legnth = 0.23
    self.gravity_ac = 9.81
    self.Ixx        = 0.0347563
    self.Iyy        = 0.0458929
    self.Izz        = 0.0977
    self.Ax         = 0.0          # drag force coefficients for velocities in the corresponding directions of the inertial frame.
    self.Ay         = 0.0
    self.Az         = 0.0
    self.k          = 8.54858e-06  # rotor_force_constant
    self.b          = 1.6e-2       # rotor_moment_constant
    
    self.dt = 0.1
    self.N = 30
    
    self.n_obstacles = 0
    
    self.x      = ca.SX.sym('x')
    self.y      = ca.SX.sym('y')
    self.z      = ca.SX.sym('z')
    self.phi    = ca.SX.sym('phi')
    self.theta  = ca.SX.sym('theta')
    self.psi    = ca.SX.sym('psi')
    self.dx     = ca.SX.sym('dx')
    self.dy     = ca.SX.sym('dy')
    self.dz     = ca.SX.sym('dz')
    self.dphi   = ca.SX.sym('dphi')
    self.dtheta = ca.SX.sym('dtheta')
    self.dpsi   = ca.SX.sym('dpsi')
    self.states = ca.vertcat(
      self.x, self.y, self.z,
      self.phi, self.theta, self.psi,
      self.dx, self.dy, self.dz,
      self.dphi, self.dtheta, self.dpsi
    )
    self.n_states = self.states.numel()
    
    self.thrust    = ca.SX.sym('thrust')
    self.tau_phi   = ca.SX.sym('tau_phi')
    self.tau_theta = ca.SX.sym('tau_theta')
    self.tau_psi   = ca.SX.sym('tau_psi')
    self.controls = ca.vertcat(
      self.thrust, self.tau_phi, self.tau_theta, self.tau_psi
    )
    self.n_controls = self.controls.numel()
    
    self.J = ca.vertcat(
      ca.horzcat( self.Ixx, 0.0, -self.Ixx * ca.sin(self.theta) ),
      ca.horzcat(0.0, self.Iyy * ca.cos(self.phi)**2 + self.Izz * ca.sin(self.phi)**2, (self.Iyy - self.Izz) * ca.cos(self.phi) * ca.sin(self.phi) * ca.cos(self.theta)),
      ca.horzcat(-self.Ixx*ca.sin(self.theta), (self.Iyy - self.Izz)*ca.cos(self.phi)*ca.sin(self.phi)*ca.cos(self.theta), self.Ixx*ca.sin(self.theta)**2 + self.Iyy*ca.sin(self.phi)**2*ca.cos(self.theta)**2 + self.Izz*ca.cos(self.phi)**2*ca.cos(self.theta)**2)
    )
    self.J_inv = ca.inv(self.J)
    
    self.C11 = 0.0
    self.C12 = (self.Iyy - self.Izz)*(self.dtheta*ca.cos(self.phi)*ca.sin(self.phi) + self.dpsi*ca.sin(self.phi)**2*ca.cos(self.theta)) + (self.Izz - self.Iyy)*self.dpsi*ca.cos(self.phi)**2*ca.cos(self.theta) - self.Ixx*self.dpsi*ca.cos(self.theta)
    self.C13 = (self.Izz - self.Iyy) * self.dpsi * ca.cos(self.phi) * ca.sin(self.phi) * ca.cos(self.theta)**2
    self.C21 = (self.Izz - self.Iyy) * (self.dtheta*ca.cos(self.phi)*ca.sin(self.phi) + self.dpsi*ca.sin(self.phi)*ca.cos(self.theta)) + (self.Iyy - self.Izz) * self.dpsi*ca.cos(self.phi)**2*ca.cos(self.theta) + self.Ixx*self.dpsi*ca.cos(self.theta)
    self.C22 = (self.Izz - self.Iyy) * self.dphi * ca.cos(self.phi) * ca.sin(self.phi)
    self.C23 = -self.Ixx*self.dpsi*ca.sin(self.theta)*ca.cos(self.theta) + self.Iyy*self.dpsi*ca.sin(self.phi)**2*ca.sin(self.theta)*ca.cos(self.theta) + self.Izz*self.dpsi*ca.cos(self.phi)**2*ca.sin(self.theta)*ca.cos(self.theta)
    self.C31 = (self.Iyy - self.Izz)*self.dpsi*ca.cos(self.theta)**2 * ca.sin(self.phi) * ca.cos(self.phi) - self.Ixx * self.dtheta * ca.cos(self.theta)
    self.C32 = (self.Izz - self.Iyy)*(self.dtheta*ca.cos(self.phi)*ca.sin(self.phi)*ca.sin(self.theta) + self.dphi*ca.sin(self.phi)**2*ca.cos(self.theta)) + (self.Iyy - self.Izz)*self.dphi*ca.cos(self.phi)**2*ca.cos(self.theta)\
                + self.Ixx*self.dpsi*ca.sin(self.theta)*ca.cos(self.theta) - self.Iyy*self.dpsi*ca.sin(self.phi)**2*ca.sin(self.theta)*ca.cos(self.theta) - self.Izz*self.dpsi*ca.cos(self.phi)**2*ca.sin(self.theta)*ca.cos(self.theta)
    self.C33 = (self.Iyy - self.Izz)*self.dphi*ca.cos(self.phi)*ca.sin(self.phi)*ca.cos(self.theta)**2 - self.Iyy*self.dtheta*ca.sin(self.phi)**2*ca.cos(self.theta)*ca.sin(self.theta) - self.Izz*self.dtheta*ca.cos(self.phi)**2*ca.cos(self.theta)*ca.sin(self.theta) + self.Ixx*self.dtheta*ca.cos(self.theta)*ca.sin(self.theta)
    
    self.C_mat = ca.vertcat(
      ca.horzcat(self.C11, self.C12, self.C13),
      ca.horzcat(self.C21, self.C22, self.C23),
      ca.horzcat(self.C31, self.C32, self.C33)
    )
    
    self.angle_dot = ca.vertcat(self.dphi, self.dtheta, self.dpsi) 
    self.cal1 = ca.mtimes(self.C_mat, self.angle_dot)
    self.cal2 = ca.vertcat(self.tau_phi, self.tau_theta, self.tau_psi) - self.cal1
    self.cal3 = ca.mtimes(self.J_inv, self.cal2)
    
    self.rhs = ca.vertcat(
      self.dx,
      self.dy,
      self.dz,
      self.dphi,
      self.dtheta,
      self.dpsi,
      (self.thrust/self.mass) * (ca.cos(self.psi) * ca.sin(self.theta) * ca.cos(self.phi) + ca.sin(self.psi) * ca.sin(self.phi)) - (self.Ax / self.mass) * self.dx,
      (self.thrust/self.mass) * (ca.sin(self.psi) * ca.sin(self.theta) * ca.cos(self.phi) - ca.cos(self.psi) * ca.sin(self.phi)) - (self.Ay / self.mass) * self.dy,
      (self.thrust/self.mass) * (ca.cos(self.theta) * ca.cos(self.phi)) - self.gravity_ac - (self.Az / self.mass) * self.dz,
      self.cal3[0],
      self.cal3[1],
      self.cal3[2]
    )
    
    self.Q_x      = 1.0
    self.Q_y      = 1.0
    self.Q_z      = 1.0
    self.Q_phi    = 0.001
    self.Q_theta  = 0.001
    self.Q_psi    = 0.001
    self.Q_dx     = 0.001
    self.Q_dy     = 0.001
    self.Q_dz     = 0.001
    self.Q_dphi   = 0.001
    self.Q_dtheta = 0.001
    self.Q_dpsi   = 0.001

    self.f = ca.Function('f', [self.states, self.controls], [self.rhs])
    self.U = ca.SX.sym('U', self.n_controls, self.N)
    self.X = ca.SX.sym('X', self.n_states, (self.N + 1))
    self.P = ca.SX.sym('P', self.n_states + self.N * (self.n_states + self.n_controls))
    self.Q = ca.diagcat(self.Q_x, self.Q_y, self.Q_z,\
                        self.Q_phi, self.Q_theta, self.Q_psi, \
                        self.Q_dx, self.Q_dy, self.Q_dz, \
                        self.Q_dphi, self.Q_dtheta, self.Q_dpsi)
    self.R = ca.diagcat(0.01, 0.01, 0.01, 0.01)
    
    
    ## lbg, ubg
    self.lbg = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_obstacles * (self.N + 1), 1))
    self.ubg = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_obstacles * (self.N + 1), 1))
    # self.lbg[self.n_states * (self.N + 1) : ] = -ca.inf   # obstacle (inequality constraint)
    
    '''
    State 최대 최소 2023-08-10
    '''
    self.x_max = ca.inf; self.x_min = -self.x_max
    self.y_max = ca.inf; self.y_min = -self.y_max
    self.z_max = ca.inf; self.z_min = 0.0
    
    self.dx_max = 0.8; self.dx_min = -self.dx_max
    self.dy_max = 0.8; self.dy_min = -self.dy_max
    self.dz_max = 0.8; self.dz_min = -self.dz_max
    
    self.phi_max   = 12.0 * ca.pi/180; self.phi_min   = -self.phi_max
    self.theta_max = 12.0 * ca.pi/180; self.theta_min = -self.theta_max
    self.psi_max   = ca.inf;           self.psi_min   = -self.psi_max
    
    self.dphi_max   = ca.pi/2; self.dphi_min   = -self.dphi_max  
    self.dtheta_max = ca.pi/2; self.dtheta_min = -self.dtheta_max
    self.dpsi_max   = ca.pi/2; self.dpsi_min   = -self.dpsi_max
    
    self.thrust_max    = 16.0; self.thrust_min    = -self.thrust_max
    self.tau_phi_max   = 10.0; self.tau_phi_min   = -self.tau_phi_max
    self.tau_theta_max = 10.0; self.tau_theta_min = -self.tau_theta_max
    self.tau_psi_max   = 10.0; self.tau_psi_min   = -self.tau_psi_max
    
    
    ## lbx, ubx
    self.lbx = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_controls * self.N), 1)
    self.ubx = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_controls * self.N), 1)
    
    self.lbx[0 : self.n_states * (self.N + 1) : self.n_states] = self.x_min
    self.ubx[0 : self.n_states * (self.N + 1) : self.n_states] = self.x_max
    self.lbx[1 : self.n_states * (self.N + 1) : self.n_states] = self.y_min
    self.ubx[1 : self.n_states * (self.N + 1) : self.n_states] = self.y_max
    self.lbx[2 : self.n_states * (self.N + 1) : self.n_states] = self.z_min
    self.ubx[2 : self.n_states * (self.N + 1) : self.n_states] = self.z_max
    
    self.lbx[3 : self.n_states * (self.N + 1) : self.n_states] = self.phi_min
    self.ubx[3 : self.n_states * (self.N + 1) : self.n_states] = self.phi_max
    self.lbx[4 : self.n_states * (self.N + 1) : self.n_states] = self.theta_min
    self.ubx[4 : self.n_states * (self.N + 1) : self.n_states] = self.theta_max
    self.lbx[5 : self.n_states * (self.N + 1) : self.n_states] = self.psi_min
    self.ubx[5 : self.n_states * (self.N + 1) : self.n_states] = self.psi_max
    
    self.lbx[6 : self.n_states * (self.N + 1) : self.n_states] = self.dx_min
    self.ubx[6 : self.n_states * (self.N + 1) : self.n_states] = self.dx_max
    self.lbx[7 : self.n_states * (self.N + 1) : self.n_states] = self.dy_min
    self.ubx[7 : self.n_states * (self.N + 1) : self.n_states] = self.dy_max
    self.lbx[8 : self.n_states * (self.N + 1) : self.n_states] = self.dz_min
    self.ubx[8 : self.n_states * (self.N + 1) : self.n_states] = self.dz_max
    
    self.lbx[9 : self.n_states * (self.N + 1) : self.n_states] = self.dphi_min
    self.ubx[9 : self.n_states * (self.N + 1) : self.n_states] = self.dphi_max
    self.lbx[10 : self.n_states * (self.N + 1) : self.n_states] = self.dtheta_min
    self.ubx[10 : self.n_states * (self.N + 1) : self.n_states] = self.dtheta_max
    self.lbx[11 : self.n_states * (self.N + 1) : self.n_states] = self.dpsi_min
    self.ubx[11 : self.n_states * (self.N + 1) : self.n_states] = self.dpsi_max
    
    self.lbx[self.n_states * (self.N + 1) : self.n_states * (self.N + 1) + self.n_controls * self.N : self.n_controls] = self.thrust_min
    self.ubx[self.n_states * (self.N + 1) : self.n_states * (self.N + 1) + self.n_controls * self.N : self.n_controls] = self.thrust_max
    self.lbx[self.n_states * (self.N + 1) + 1 : self.n_states * (self.N + 1) + self.n_controls * self.N : self.n_controls] = self.tau_phi_min
    self.ubx[self.n_states * (self.N + 1) + 1 : self.n_states * (self.N + 1) + self.n_controls * self.N : self.n_controls] = self.tau_phi_max
    self.lbx[self.n_states * (self.N + 1) + 2 : self.n_states * (self.N + 1) + self.n_controls * self.N : self.n_controls] = self.tau_theta_min
    self.ubx[self.n_states * (self.N + 1) + 2 : self.n_states * (self.N + 1) + self.n_controls * self.N : self.n_controls] = self.tau_theta_max
    self.lbx[self.n_states * (self.N + 1) + 3 : self.n_states * (self.N + 1) + self.n_controls * self.N : self.n_controls] = self.tau_psi_min
    self.ubx[self.n_states * (self.N + 1) + 3 : self.n_states * (self.N + 1) + self.n_controls * self.N : self.n_controls] = self.tau_psi_max
    
    self.args = {
      'lbg' : self.lbg,
      'ubg' : self.ubg,
      'lbx' : self.lbx,
      'ubx' : self.ubx
    }
    
    # param_robot_name = rospy.search_param('robot_name')
    # self.robot_name = '/'
    # self.robot_name += rospy.get_param(param_robot_name)
    self.robot_name = 'UAV1'
    
    
    x_init = rospy.search_param('init_position' + self.robot_name + '_x')
    y_init = rospy.search_param('init_position' + self.robot_name + '_y')
    self.x_init = rospy.get_param(x_init)
    self.y_init = rospy.get_param(y_init)
    
    self.x_init = 0.0; self.y_init = 0.0;  self.z_init = 2.0
    
    self.phi_init = 0.0;  self.theta_init = 0.0;  self.psi_init = 0.0
    self.dx_init = 0.0;   self.dy_init = 0.0;     self.dz_init = 0.0
    self.dphi_init = 0.0; self.dtheta_init = 0.0; self.dpsi_init = 0.0
    
    self.state_init = ca.DM([self.x_init, self.y_init, self.theta_init, self.phi_init, self.theta_init, self.psi_init, self.dx_init, self.dy_init, self.dz_init, self.dphi_init, self.dtheta_init, self.dpsi_init])
    
    self.u0 = ca.DM.zeros(self.N, self.n_controls)
    self.X0 = ca.repmat(self.state_init, 1, self.N + 1)
    
  '''
  function of MPC
  '''
  def while_loop_condition(self):
    return not rospy.is_shutdown()
  
  def state_update(self, pos, ori, dpos, dori):
    self.state_init[0 : 3]  = pos
    self.state_init[3 : 6]  = ori
    self.state_init[6 : 9]  = dpos
    self.state_init[9 : 12] = dori
    
  def setting_solver(self):
    
    self.obj = 0.0
    self.g = []
    self.g = self.X[:, 0] - self.P[ : self.n_states]
    
    for k in range(self.N):
      self.st  = self.X[:, k]
      self.con = self.U[:, k]
      
      state_error_   = self.st  - self.P[(self.n_states + self.n_controls) * k + 12 : (self.n_states + self.n_controls) * k + 24]
      control_error_ = self.con - self.P[(self.n_states + self.n_controls) * k + 24 : (self.n_states + self.n_controls) * k + 28]
      
      self.obj = self.obj + ca.mtimes([state_error_.T, self.Q, state_error_]) + ca.mtimes([control_error_.T, self.R, control_error_])
      
      self.st_next = self.X[:, k + 1]
      self.f_value = self.f(self.st, self.con)
      
      self.st_next_euler = self.st + (self.dt * self.f_value)
      self.g = ca.vertcat(self.g, self.st_next - self.st_next_euler)
      
    if self.n_obstacles != 0:
      self.collision_avoidance()
      
    self.OPT_variables = ca.vertcat(
      self.X.reshape((-1, 1)),
      self.U.reshape((-1, 1))
    )
    
    self.nlp_prob = {
      'f' : self.obj,
      'x' : self.OPT_variables,
      'g' : self.g,
      'p' : self.P
    }
    
    '''
    IPOPT (Interior Point Optimizer) is an open source software package for large-scale nonlinear optimization.
    It can be used to solve general nonlinear programming problems (NLPs)
    '''
    
    self.opts = {
      'ipopt' : {
        'max_iter' : 2000,
        'print_level' : 0,
        'acceptable_tol' : 1e-8,
        'acceptable_obj_change_tol' : 1e-6
      },
      'print_time' : 0
    }
    
    self.solver = ca.nlpsol('solver', 'ipopt', self.nlp_prob, self.opts)
    
  def make_args_p(self):
    self.args['p'] = ca.DM.zeros((self.n_states + self.n_controls) * self.N + self.n_states)
    self.args['p'][0 : 12] = self.state_init
  
  def setting_reference(self, ref):
    
    for k in range(self.N):
      
      q = np.array([ref[k].pose.orientation.x, ref[k].pose.orientation.y, ref[k].pose.orientation.z, ref[k].pose.orientation.w])
      q_normalized = q / np.linalg.norm(q)
      
      euler_angles = euler_from_quaternion(q_normalized)
      
      self.args['p'][(self.n_states + self.n_controls) * k + 12] = ref[k].pose.position.x
      self.args['p'][(self.n_states + self.n_controls) * k + 13] = ref[k].pose.position.y
      self.args['p'][(self.n_states + self.n_controls) * k + 14] = ref[k].pose.position.z
      self.args['p'][(self.n_states + self.n_controls) * k + 15] = euler_angles[0]
      self.args['p'][(self.n_states + self.n_controls) * k + 16] = euler_angles[1]
      self.args['p'][(self.n_states + self.n_controls) * k + 17] = euler_angles[2]
      self.args['p'][(self.n_states + self.n_controls) * k + 18] = 0.0
      self.args['p'][(self.n_states + self.n_controls) * k + 19] = 0.0
      self.args['p'][(self.n_states + self.n_controls) * k + 20] = 0.0
      self.args['p'][(self.n_states + self.n_controls) * k + 21] = 0.0
      self.args['p'][(self.n_states + self.n_controls) * k + 22] = 0.0
      self.args['p'][(self.n_states + self.n_controls) * k + 23] = 0.0
      
  def reshape_and_init_opt_variables(self):
    self.args['x0'] = ca.vertcat(
      ca.reshape(self.X0, self.n_states * (self.N + 1), 1),
      ca.reshape(self.u0, self.n_controls * self.N, 1)
    )
    
  def call_solver(self):
    self.sol = self.solver(
      x0 = self.args['x0'],
      lbx = self.args['lbx'],
      ubx = self.args['ubx'],
      lbg = self.args['lbg'],
      ubg = self.args['ubg'],
      p = self.args['p']
    )
    
    
  def get_result(self):
    self.X0 = ca.reshape( self.sol['x'][ : self.n_states * (self.N + 1)], self.n_states, self.N + 1 )
    self.u0 = ca.reshape( self.sol['x'][self.n_states * (self.N + 1) : ], self.n_controls, self.N )
    # print('predict_state', self.X0[:, 8][0], self.X0[:, 8][1], self.X0[:, 8][2])
    # print('control_input', self.u0[:, 0])
    
    return self.X0, self.u0
    
  def shift_(self):
    self.X0, self.u0 = shift(self.X0, self.u0)
    
  
  def collision_avoidance(self):
    pass
      


class ROS():
  def __init__(self, GPS):
    rospy.init_node('CPS_UAV_MPC')
    
    # param_robot_name = rospy.search_param('robot_name')
    # self.robot_name = '/'
    # self.robot_name += rospy.get_param(param_robot_name)
    self.robot_name = 'UAV1'
    
    self.state_sub = rospy.Subscriber(self.robot_name + '/mavros/state', State, self.state_callback, queue_size = 1)
    self.odom_sub  = rospy.Subscriber(self.robot_name + '/mavros/local_position/odom', PoseStamped, self.odom_callback, queue_size = 1)
    
    if GPS:
      self.ref_sub = rospy.Subscriber(self.robot_name + '/GPS_based_ref_path', Path, self.ref_path_callback, queue_size = 1)
    else:
      self.ref_sub = rospy.Subscriber(self.robot_name + '/ref_path', Path, self.ref_path_callback, queue_size = 1)
    
    self.setpoint_pub = rospy.Publisher(self.robot_name + '/mavros/setpoint_position/local', PoseStamped, queue_size = 1)
    self.setpoint_visu_pub = rospy.Publisher(self.robot_name + '/setpoint/visual', Marker, queue_size = 1)
    
    self.predict_path_pub = rospy.Publisher(self.robot_name + '/predict_path', Path, queue_size = 1)
    
    self.state = State()
    self.ref_path = Path()
    self.state_queue = Queue()
    self.ref_path_queue = Queue()
    
    self.robot_pos  = [0.0, 0.0, 0.0]
    self.robot_ori  = [0.0, 0.0, 0.0]
    self.robot_dpos = [0.0, 0.0, 0.0]
    self.robot_dori = [0.0, 0.0, 0.0]
    
  def state_callback(self, msg):
    self.state_queue.put(msg)
    
  def odom_callback(self, msg):
    self.robot_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    
    q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    q_normalized = q / np.linalg.norm(q)
    
    self.robot_ori = euler_from_quaternion(q_normalized)
    
    self.robot_dpos = [msg.twist.twist.linear.x,  msg.twist.twist.linear.y,  msg.twist.twist.linear.z]
    self.robot_dori = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
    
  def ref_path_callback(self, msg):
    self.ref_path_queue.put(msg)
    self.ref_path = msg
    
  
  def setpoint_publish(self, X0, k):
    pub_msg = PoseStamped()
    
    pub_msg.header.frame_id = 'map'
    pub_msg.header.stamp = rospy.Time.now()
    
    roll  = X0[:, k][3]
    pitch = X0[:, k][4]
    yaw   = X0[:, k][5]
    
    q = quaternion_from_euler(roll, pitch, yaw)
    q_normaliazed = q / np.linalg.norm(q)
    
    pub_msg.pose.position.x    = X0[:, k][0]
    pub_msg.pose.position.y    = X0[:, k][1]
    pub_msg.pose.position.z    = X0[:, k][2]
    pub_msg.pose.orientation.x = q_normaliazed[0]
    pub_msg.pose.orientation.y = q_normaliazed[1]
    pub_msg.pose.orientation.z = q_normaliazed[2]
    pub_msg.pose.orientation.w = q_normaliazed[3]
    
    self.setpoint_pub.publish(pub_msg)
    
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    
    marker.id = 1
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.color = ColorRGBA(1, 0, 0, 0.5)
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    
    marker.points.append(Point(X0[:, k][0], X0[:, k][1], X0[:, k][2]))
    
    self.setpoint_visu_pub.publish(marker)
    
  def predict_path_publish(self, X0, N_):
    predict_path = Path()
    
    predict_path.header.frame_id = 'map'
    predict_path.header.stamp = rospy.Time.now()
    
    for i in range(N_):
      pose_stamped = PoseStamped()
      pose_stamped.pose.position.x = X0[:, i][0]
      pose_stamped.pose.position.y = X0[:, i][1]
      pose_stamped.pose.position.z = X0[:, i][2]
      predict_path.poses.append(pose_stamped)
      
    self.predict_path_pub.publish(predict_path)
    
    
    
def main():
  
  N_ = 30
  GPS_ = False
  
  ros = ROS(GPS_)
  mpc = MPC()
  
  r = rospy.Rate(10)
  loop_cnt = 0
  
  start_time = timeit.default_timer()
  
  while mpc.while_loop_condition():
    
    if ros.state_queue.empty():
      print(ros.robot_name + ' state is not subscribed..')
      continue
    
    if ros.ref_path_queue.empty():
      print(ros.robot_name + ' reference trajectory is not subscribed..')
      continue
    
    pos = ros.robot_pos
    ori = ros.robot_ori
    dpos = ros.robot_dpos
    dori = ros.robot_dori
    
    reference_trajectory = ros.ref_path.poses
    
    mpc_start_time = timeit.default_timer()
    
    mpc.state_update(pos, ori, dpos, dori)
    mpc.setting_solver()
    mpc.make_args_p()
    mpc.setting_reference(reference_trajectory)
    mpc.reshape_and_init_opt_variables()
    mpc.call_solver()
    result_X0, result_u0 = mpc.get_result()

    mpc_finish_time = timeit.default_timer()
    
    print('Runtime : {:.5f} s'.format(mpc_finish_time - mpc_start_time))
    
    ros.predict_path_publish(result_X0, N_)
    ros.setpoint_publish(result_X0, 8)
    mpc.shift_()
    
if __name__ == '__main__':
  main()