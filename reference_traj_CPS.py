#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@MPC trajectory Maker _ CPS

@file reference_traj_CPS.py
@author Jeong-Uk Lee (ju.lee@sch.ac.kr)
@version 0.1
@date 2023-08-10 16:04

"""

import numpy as np

import signal
import sys
import rospy
import timeit

from multiprocessing import Queue

from nav_msgs.msg import Path
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

RED = '\033[31m'
GREEN = '\033[32m'
BLUE = '\033[34m'
END = '\033[0m'

'''
Ctrl+C : program exit
'''

def signal_handler(signal, frame):
  print('\nYou pressed Ctrl+C!')
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class ROS():
  def __init__(self):
    
    rospy.init_node('CPS_Reference_Maker')
    
    self.robot1_state_queue = Queue(); self.robot2_state_queue = Queue(); self.robot3_state_queue = Queue()
    self.robot1_state_sub = rospy.Subscriber('UAV1/mavros/state', State, self.state1_callback, queue_size = 1)
    self.robot2_state_sub = rospy.Subscriber('UAV2/mavros/state', State, self.state2_callback, queue_size = 1)
    self.robot3_state_sub = rospy.Subscriber('UAV3/mavros/state', State, self.state3_callback, queue_size = 1)
    
    self.robot1_local_pos = PoseStamped(); self.robot2_local_pos = PoseStamped(); self.robot3_local_pos = PoseStamped()
    self.robot1_local_queue = Queue()    ; self.robot2_local_queue = Queue()    ; self.robot3_local_queue = Queue()
    self.robot1_local_pos_sub = rospy.Subscriber('UAV1/mavros/local_position/pose', PoseStamped, self.local_pos1_callback, queue_size = 1)
    self.robot2_local_pos_sub = rospy.Subscriber('UAV2/mavros/local_position/pose', PoseStamped, self.local_pos2_callback, queue_size = 1)
    self.robot3_local_pos_sub = rospy.Subscriber('UAV3/mavros/local_position/pose', PoseStamped, self.local_pos3_callback, queue_size = 1)
    
    self.reference_pub = rospy.Publisher('Reference_path', Path, queue_size = 1)
    self.robot1_path_pub = rospy.Publisher('UAV1/ref_path', Path, queue_size = 1)
    self.robot2_path_pub = rospy.Publisher('UAV2/ref_path', Path, queue_size = 1)
    self.robot3_path_pub = rospy.Publisher('UAV3/ref_path', Path, queue_size = 1)
    
    self.reference_GPS_based_pub = rospy.Publisher('Reference_GPS_based_path', Path, queue_size = 1)
    self.robot1_GPS_based_path_pub = rospy.Publisher('UAV1/GPS_based_ref_path', Path, queue_size = 1)
    self.robot2_GPS_based_path_pub = rospy.Publisher('UAV2/GPS_based_ref_path', Path, queue_size = 1)
    self.robot3_GPS_based_path_pub = rospy.Publisher('UAV3/GPS_based_ref_path', Path, queue_size = 1)
    
    
  def state1_callback(self, msg):
    self.robot1_state_queue.put(msg)
  
  def state2_callback(self, msg):
    self.robot2_state_queue.put(msg)
    
  def state3_callback(self, msg):
    self.robot3_state_queue.put(msg)
    
  def local_pos1_callback(self, msg):
    self.robot1_local_queue.put(msg)
    
    self.robot1_position    = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    self.robot1_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    
  def local_pos2_callback(self, msg):
    self.robot2_local_queue.put(msg)
    
    self.robot2_position    = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    self.robot2_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    
  def local_pos3_callback(self, msg):
    self.robot3_local_queue.put(msg)
    
    self.robot3_position    = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    self.robot3_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
  
  
  def averaging_poses(self):
    
    r = rospy.Rate(10)
    
    robot1_local_pos_history = []
    robot2_local_pos_history = []
    robot3_local_pos_history = []
    self.robot1_local_pos_mean = np.zeros(7)
    self.robot2_local_pos_mean = np.zeros(7)
    self.robot3_local_pos_mean = np.zeros(7)
    
    while 1:
    
      robot1_local_pos_history.append([self.robot1_position[0], self.robot1_position[1], self.robot1_position[2], \
                                      self.robot1_orientation[0], self.robot1_orientation[1], self.robot1_orientation[2], self.robot1_orientation[3]])
      robot2_local_pos_history.append([self.robot2_position[0], self.robot2_position[1], self.robot2_position[2], \
                                      self.robot2_orientation[0], self.robot2_orientation[1], self.robot2_orientation[2], self.robot2_orientation[3]])
      robot3_local_pos_history.append([self.robot3_position[0], self.robot3_position[1], self.robot3_position[2], \
                                      self.robot3_orientation[0], self.robot3_orientation[1], self.robot3_orientation[2], self.robot3_orientation[3]])
      
      if len(robot1_local_pos_history) < 51:
        print('Averaging_poses')
        r.sleep()
        continue
      else:
        mean_x1 = 0.0; mean_y1 = 0.0; mean_z1 = 0.0; mean_ori_x1 = 0.0; mean_ori_y1 = 0.0; mean_ori_z1 = 0.0; mean_ori_w1 = 0.0
        mean_x2 = 0.0; mean_y2 = 0.0; mean_z2 = 0.0; mean_ori_x2 = 0.0; mean_ori_y2 = 0.0; mean_ori_z2 = 0.0; mean_ori_w2 = 0.0
        mean_x3 = 0.0; mean_y3 = 0.0; mean_z3 = 0.0; mean_ori_x3 = 0.0; mean_ori_y3 = 0.0; mean_ori_z3 = 0.0; mean_ori_w3 = 0.0
        
        for i in range(len(robot1_local_pos_history)):
          mean_x1     += robot1_local_pos_history[i][0]
          mean_y1     += robot1_local_pos_history[i][1]
          mean_z1     += robot1_local_pos_history[i][2]
          mean_ori_x1 += robot1_local_pos_history[i][3]
          mean_ori_y1 += robot1_local_pos_history[i][4]
          mean_ori_z1 += robot1_local_pos_history[i][5]
          mean_ori_w1 += robot1_local_pos_history[i][6]
          
          mean_x2     += robot2_local_pos_history[i][0]
          mean_y2     += robot2_local_pos_history[i][1]
          mean_z2     += robot2_local_pos_history[i][2]
          mean_ori_x2 += robot2_local_pos_history[i][3]
          mean_ori_y2 += robot2_local_pos_history[i][4]
          mean_ori_z2 += robot2_local_pos_history[i][5]
          mean_ori_w2 += robot2_local_pos_history[i][6]
          
          mean_x3     += robot3_local_pos_history[i][0]
          mean_y3     += robot3_local_pos_history[i][1]
          mean_z3     += robot3_local_pos_history[i][2]
          mean_ori_x3 += robot3_local_pos_history[i][3]
          mean_ori_y3 += robot3_local_pos_history[i][4]
          mean_ori_z3 += robot3_local_pos_history[i][5]
          mean_ori_w3 += robot3_local_pos_history[i][6]
          
        mean_x1     = mean_x1 / len(robot1_local_pos_history)
        mean_y1     = mean_y1 / len(robot1_local_pos_history)
        mean_z1     = mean_z1 / len(robot1_local_pos_history)
        mean_ori_x1 = mean_ori_x1 / len(robot1_local_pos_history)
        mean_ori_y1 = mean_ori_y1 / len(robot1_local_pos_history)
        mean_ori_z1 = mean_ori_z1 / len(robot1_local_pos_history)
        mean_ori_w1 = mean_ori_w1 / len(robot1_local_pos_history)
        
        mean_x2     = mean_x2 / len(robot2_local_pos_history)
        mean_y2     = mean_y2 / len(robot2_local_pos_history)
        mean_z2     = mean_z2 / len(robot2_local_pos_history)
        mean_ori_x2 = mean_ori_x2 / len(robot2_local_pos_history)
        mean_ori_y2 = mean_ori_y2 / len(robot2_local_pos_history)
        mean_ori_z2 = mean_ori_z2 / len(robot2_local_pos_history)
        mean_ori_w2 = mean_ori_w2 / len(robot2_local_pos_history)
        
        mean_x3     = mean_x3 / len(robot3_local_pos_history)
        mean_y3     = mean_y3 / len(robot3_local_pos_history)
        mean_z3     = mean_z3 / len(robot3_local_pos_history)
        mean_ori_x3 = mean_ori_x3 / len(robot3_local_pos_history)
        mean_ori_y3 = mean_ori_y3 / len(robot3_local_pos_history)
        mean_ori_z3 = mean_ori_z3 / len(robot3_local_pos_history)
        mean_ori_w3 = mean_ori_w3 / len(robot3_local_pos_history)
        
        self.robot1_local_pos_mean[0] = mean_x1
        self.robot1_local_pos_mean[1] = mean_y1
        self.robot1_local_pos_mean[2] = mean_z1
        self.robot1_local_pos_mean[3] = mean_ori_x1
        self.robot1_local_pos_mean[4] = mean_ori_y1
        self.robot1_local_pos_mean[5] = mean_ori_z1
        self.robot1_local_pos_mean[6] = mean_ori_w1
        
        self.robot2_local_pos_mean[0] = mean_x2
        self.robot2_local_pos_mean[1] = mean_y2
        self.robot2_local_pos_mean[2] = mean_z2
        self.robot2_local_pos_mean[3] = mean_ori_x2
        self.robot2_local_pos_mean[4] = mean_ori_y2
        self.robot2_local_pos_mean[5] = mean_ori_z2
        self.robot2_local_pos_mean[6] = mean_ori_w2
        
        self.robot3_local_pos_mean[0] = mean_x3
        self.robot3_local_pos_mean[1] = mean_y3
        self.robot3_local_pos_mean[2] = mean_z3
        self.robot3_local_pos_mean[3] = mean_ori_x3
        self.robot3_local_pos_mean[4] = mean_ori_y3
        self.robot3_local_pos_mean[5] = mean_ori_z3
        self.robot3_local_pos_mean[6] = mean_ori_w3
          
        print('Averaging_poses is completed!')
        break
      
  def rotation(self, point, center):
    
    result = np.zeros(3)
    
    q = np.array([self.robot1_local_pos_mean[3], self.robot1_local_pos_mean[4], self.robot1_local_pos_mean[5], self.robot1_local_pos_mean[6]])
    q_normalized = q / np.linalg.norm(q)
    
    roll, pitch, yaw = euler_from_quaternion(q_normalized)
    
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                                [np.sin(yaw), np.cos(yaw)]])
    
    #### Robot1 기준으로 각도 구하고 rotation matrix 생성
    #### 아래부터는 각 포인트를 위에서 구한 각도를 기준으로 회전
    
    trans_x = point[0] - center[0]
    trans_y = point[1] - center[1]
    
    trans_ = (trans_x, trans_y)
    
    rotated_point = np.dot(rotation_matrix, trans_) + (center[0], center[1])
    
    result[0] = rotated_point[0]
    result[1] = rotated_point[1]
    result[2] = point[2]
    
    return result
  
  
  def reference_publish(self, N, ref_path):
    ref_trajectory = Path()
    
    ref_trajectory.header.frame_id = 'map'
    ref_trajectory.header.stamp = rospy.Time.now()
    
    for i in range(N):
      pose_stamped = PoseStamped()
      pose_stamped.pose.position.x = ref_path[i][0]
      pose_stamped.pose.position.y = ref_path[i][1]
      pose_stamped.pose.position.z = ref_path[i][2]
      ref_trajectory.poses.append(pose_stamped)
      
  def UAV_reference_publish(self, N_, robot_1, robot_2, robot_3):
    robot1_trajectory = Path()
    robot2_trajectory = Path()
    robot3_trajectory = Path()
    
    robot1_trajectory.header.frame_id = 'map'
    robot1_trajectory.header.stamp = rospy.Time.now()
    robot2_trajectory.header.frame_id = 'map'
    robot2_trajectory.header.stamp = rospy.Time.now()
    robot3_trajectory.header.frame_id = 'map'
    robot3_trajectory.header.stamp = rospy.Time.now()
    
    ref_roll = 0.0; ref_pitch = 0.0; ref_yaw = 0.0
    q = quaternion_from_euler(ref_roll, ref_pitch, ref_yaw)
    q_normalized = q / np.linalg.norm(q)
    
    for i in range(N_):
      pose_stamped = PoseStamped()
      pose_stamped.pose.position.x    = robot_1[i][0]
      pose_stamped.pose.position.y    = robot_1[i][1]
      pose_stamped.pose.position.z    = robot_1[i][2]
      pose_stamped.pose.orientation.x = q_normalized[0]
      pose_stamped.pose.orientation.y = q_normalized[1]
      pose_stamped.pose.orientation.z = q_normalized[2]
      pose_stamped.pose.orientation.w = q_normalized[3]
      robot1_trajectory.poses.append(pose_stamped)
    
    for i in range(N_):
      pose_stamped = PoseStamped()
      pose_stamped.pose.position.x    = robot_2[i][0]
      pose_stamped.pose.position.y    = robot_2[i][1]
      pose_stamped.pose.position.z    = robot_2[i][2]
      pose_stamped.pose.orientation.x = q_normalized[0]
      pose_stamped.pose.orientation.y = q_normalized[1]
      pose_stamped.pose.orientation.z = q_normalized[2]
      pose_stamped.pose.orientation.w = q_normalized[3]
      robot2_trajectory.poses.append(pose_stamped)
      
    for i in range(N_):
      pose_stamped = PoseStamped()
      pose_stamped.pose.position.x    = robot_3[i][0]
      pose_stamped.pose.position.y    = robot_3[i][1]
      pose_stamped.pose.position.z    = robot_3[i][2]
      pose_stamped.pose.orientation.x = q_normalized[0]
      pose_stamped.pose.orientation.y = q_normalized[1]
      pose_stamped.pose.orientation.z = q_normalized[2]
      pose_stamped.pose.orientation.w = q_normalized[3]
      robot3_trajectory.poses.append(pose_stamped)
    
    self.robot1_path_pub.publish(robot1_trajectory)
    self.robot2_path_pub.publish(robot2_trajectory)
    self.robot3_path_pub.publish(robot3_trajectory)
    
  def UAV_GPS_based_reference_publish(self, N_, robot_1, robot_2, robot_3):
    robot1_trajectory = Path()
    robot2_trajectory = Path()
    robot3_trajectory = Path()
    
    robot1_trajectory.header.frame_id = 'map'
    robot1_trajectory.header.stamp = rospy.Time.now()
    robot2_trajectory.header.frame_id = 'map'
    robot2_trajectory.header.stamp = rospy.Time.now()
    robot3_trajectory.header.frame_id = 'map'
    robot3_trajectory.header.stamp = rospy.Time.now()
    
    ref_roll = 0.0; ref_pitch = 0.0; ref_yaw = 0.0
    q = quaternion_from_euler(ref_roll, ref_pitch, ref_yaw)
    q_normalized = q / np.linalg.norm(q)
    
    robot1_start = [self.robot1_local_pos_mean[0], self.robot1_local_pos_mean[1]]
    robot2_start = [self.robot2_local_pos_mean[0], self.robot2_local_pos_mean[1]]
    robot3_start = [self.robot3_local_pos_mean[0], self.robot3_local_pos_mean[1]]
    
    for i in range(N_):
      pose_stamped = PoseStamped()
      rotated_point = self.rotation(robot_1[i], robot1_start)
      pose_stamped.pose.position.x    = rotated_point[0]
      pose_stamped.pose.position.y    = rotated_point[1]
      pose_stamped.pose.position.z    = rotated_point[2]
      pose_stamped.pose.orientation.x = q_normalized[0] + self.robot1_local_pos_mean[3]
      pose_stamped.pose.orientation.y = q_normalized[1] + self.robot1_local_pos_mean[4]
      pose_stamped.pose.orientation.z = q_normalized[2] + self.robot1_local_pos_mean[5]
      pose_stamped.pose.orientation.w = q_normalized[3] + self.robot1_local_pos_mean[6]
      robot1_trajectory.poses.append(pose_stamped)
      
    for i in range(N_):
      pose_stamped = PoseStamped()
      rotated_point = self.rotation(robot_2[i], robot2_start)
      pose_stamped.pose.position.x    = rotated_point[0]
      pose_stamped.pose.position.y    = rotated_point[1]
      pose_stamped.pose.position.z    = rotated_point[2]
      pose_stamped.pose.orientation.x = q_normalized[0] + self.robot2_local_pos_mean[3]
      pose_stamped.pose.orientation.y = q_normalized[1] + self.robot2_local_pos_mean[4]
      pose_stamped.pose.orientation.z = q_normalized[2] + self.robot2_local_pos_mean[5]
      pose_stamped.pose.orientation.w = q_normalized[3] + self.robot2_local_pos_mean[6]
      robot2_trajectory.poses.append(pose_stamped)
      
    for i in range(N_):
      pose_stamped = PoseStamped()
      rotated_point = self.rotation(robot_3[i], robot3_start)
      pose_stamped.pose.position.x    = rotated_point[0]
      pose_stamped.pose.position.y    = rotated_point[1]
      pose_stamped.pose.position.z    = rotated_point[2]
      pose_stamped.pose.orientation.x = q_normalized[0] + self.robot3_local_pos_mean[3]
      pose_stamped.pose.orientation.y = q_normalized[1] + self.robot3_local_pos_mean[4]
      pose_stamped.pose.orientation.z = q_normalized[2] + self.robot3_local_pos_mean[5]
      pose_stamped.pose.orientation.w = q_normalized[3] + self.robot3_local_pos_mean[6]
      robot3_trajectory.poses.append(pose_stamped)
    
    self.robot1_GPS_based_path_pub.publish(robot1_trajectory)
    self.robot2_GPS_based_path_pub.publish(robot2_trajectory)
    self.robot3_GPS_based_path_pub.publish(robot3_trajectory)
        
  
class Formation():
  def __init__(self, N_, dt, nominal_speed):
    
    self.N = N_
    self.dt = dt
    self.nominal_speed = nominal_speed
    
    self.reference_ = [[0.0, 0.0, 0.0] for _ in range(N_)]
    
    self.robot1_ref = [[0.0, 0.0, 0.0] for _ in range(N_)]
    self.robot2_ref = [[0.0, 0.0, 0.0] for _ in range(N_)]
    self.robot3_ref = [[0.0, 0.0, 0.0] for _ in range(N_)]
    
  def ReferenceTrajectoryGenerator(self, t):
    
    for i in range(self.N):
      self.reference_[i][0] = (t + i * self.dt)
      self.reference_[i][1] = 0.0
      self.reference_[i][2] = 2.0
      
    return self.reference_
  

  def UAV_TrajectoryGenerator(self, running_time, formation_change_time):
    
    change_time = formation_change_time
    
    if running_time < change_time * 2:
      for i in range(self.N):
        self.robot1_ref[i][0] = self.reference_[i][0]
        self.robot1_ref[i][1] = self.reference_[i][1]
        self.robot1_ref[i][2] = self.reference_[i][2]
        
        self.robot2_ref[i][0] = self.reference_[i][0]
        self.robot2_ref[i][1] = self.reference_[i][1] + 5.0
        self.robot2_ref[i][2] = self.reference_[i][2]
        
        self.robot3_ref[i][0] = self.reference_[i][0]
        self.robot3_ref[i][1] = self.reference_[i][1] - 5.0
        self.robot3_ref[i][2] = self.reference_[i][2]
        
    elif running_time >= change_time * 2 and running_time < change_time * 3:
      for i in range(self.N):
        self.robot1_ref[i][0] = self.reference_[i][0]
        self.robot1_ref[i][1] = self.reference_[i][1]
        self.robot1_ref[i][2] = self.reference_[i][2]
        
        self.robot2_ref[i][0] = self.reference_[i][0] - 2.0
        self.robot2_ref[i][1] = self.reference_[i][1] + 5.0
        self.robot2_ref[i][2] = self.reference_[i][2]
        
        self.robot3_ref[i][0] = self.reference_[i][0] - 2.0
        self.robot3_ref[i][1] = self.reference_[i][1] - 5.0
        self.robot3_ref[i][2] = self.reference_[i][2]
        
    elif running_time >= change_time * 3 and running_time < change_time * 4:
      for i in range(self.N):
        self.robot1_ref[i][0] = self.reference_[i][0]
        self.robot1_ref[i][1] = self.reference_[i][1]
        self.robot1_ref[i][2] = self.reference_[i][2]
        
        self.robot2_ref[i][0] = self.reference_[i][0]
        self.robot2_ref[i][1] = self.reference_[i][1] + 5.0
        self.robot2_ref[i][2] = self.reference_[i][2]
        
        self.robot3_ref[i][0] = self.reference_[i][0]
        self.robot3_ref[i][1] = self.reference_[i][1] - 5.0
        self.robot3_ref[i][2] = self.reference_[i][2]
        
    elif running_time >= change_time * 4 and running_time < change_time * 5:
      for i in range(self.N):
        self.robot1_ref[i][0] = self.reference_[i][0]
        self.robot1_ref[i][1] = self.reference_[i][1]
        self.robot1_ref[i][2] = self.reference_[i][2]
        
        self.robot2_ref[i][0] = self.reference_[i][0]
        self.robot2_ref[i][1] = self.reference_[i][1] + 5.0
        self.robot2_ref[i][2] = self.reference_[i][2] + 2.0
        
        self.robot3_ref[i][0] = self.reference_[i][0]
        self.robot3_ref[i][1] = self.reference_[i][1] - 5.0
        self.robot3_ref[i][2] = self.reference_[i][2] + 2.0
        
    elif running_time >= change_time * 5 and running_time < change_time * 6:
      for i in range(self.N):
        self.robot1_ref[i][0] = self.reference_[i][0]
        self.robot1_ref[i][1] = self.reference_[i][1]
        self.robot1_ref[i][2] = self.reference_[i][2]
        
        self.robot2_ref[i][0] = self.reference_[i][0]
        self.robot2_ref[i][1] = self.reference_[i][1] + 5.0
        self.robot2_ref[i][2] = self.reference_[i][2]
        
        self.robot3_ref[i][0] = self.reference_[i][0]
        self.robot3_ref[i][1] = self.reference_[i][1] - 5.0
        self.robot3_ref[i][2] = self.reference_[i][2]
        
    elif running_time >= change_time * 6:
      for i in range(self.N):
        self.robot1_ref[i][0] = self.reference_[i][0]
        self.robot1_ref[i][1] = self.reference_[i][1]
        self.robot1_ref[i][2] = self.reference_[i][2] - 1.0
        
        self.robot2_ref[i][0] = self.reference_[i][0]
        self.robot2_ref[i][1] = self.reference_[i][1] + 5.0
        self.robot2_ref[i][2] = self.reference_[i][2] - 1.0
        
        self.robot3_ref[i][0] = self.reference_[i][0]
        self.robot3_ref[i][1] = self.reference_[i][1] - 5.0
        self.robot3_ref[i][2] = self.reference_[i][2] - 1.0
        
    return self.robot1_ref, self.robot2_ref, self.robot3_ref
    

def main():
  print(__file__ + ' Start !!')
  
  Robot1 = True
  Robot2 = True
  Robot3 = True
  
  GPS_based = False
  
  start_switch = True
  
  N_ = 30
  dt = 0.1
  nominal_speed = 0.5 #[m/s]
  
  formation_change_time = 10.0
  formation_change_cnt = 6
  past_time = 0.0
  loop_cnt = 0
  t = 0
  
  ros = ROS()
  formation = Formation(N_, dt, nominal_speed)
  
  while 1:
    if ros.robot1_state_queue.empty() and Robot1:
      print('UAV1 state is not subscribed..')
      continue
    elif ros.robot2_state_queue.empty() and Robot2:
      print('UAV2 state is not subscribed..')
      continue
    elif ros.robot3_state_queue.empty() and Robot3:
      print('UAV3 state is not subscribed..')
      continue
    else: break
    
  while 1:
    if ros.robot1_local_queue.empty() and Robot1:
      print('UAV1 local pose is not subscribed..')
      continue
    elif ros.robot2_local_queue.empty() and Robot2:
      print('UAV2 local pose is not subscribed..')
      continue
    elif ros.robot3_local_queue.empty() and Robot3:
      print('UAV3 local pose is not subscribed..')
      continue
    else: break
    
    
  if start_switch:
    print('\n')
    print('*' * 45)
    input(GREEN + 'Enter any number to start when ' + END + RED + 'GPS' + END + GREEN + ' is stabilized\n' + END)
    start_switch = False
  
  ros.averaging_poses()
  
  r = rospy.Rate(1.0 / dt)
  start_time = timeit.default_timer()

  
  while not rospy.is_shutdown():
    running_time = timeit.default_timer() - start_time
    
    if running_time < formation_change_time:   # 10초 동안 Hovering
      t = 0
    
    if running_time - past_time >= 1.0:
      print('\n')
      print('Running time   : {:.2f} [s]'.format(running_time))
      
      if running_time < formation_change_time:
        print('Formation type : ' + GREEN + 'Hovering.. 2m' + END)
      elif running_time > formation_change_time and running_time <= formation_change_time * 2:
        print('Formation type : ' + GREEN + 'Formation 1' + END)
      elif running_time > formation_change_time * 2 and running_time <= formation_change_time * 3:
        print('Formation type : ' + GREEN + 'Formation 2' + END)
      elif running_time > formation_change_time * 3 and running_time <= formation_change_time * 4:
        print('Formation type : ' + GREEN + 'Formation 1' + END)
      elif running_time > formation_change_time * 4 and running_time <= formation_change_time * 5:
        print('Formation type : ' + GREEN + 'Formation 3' + END)
      elif running_time > formation_change_time * 5 and running_time <= formation_change_time * 6:
        print('Formation type : ' + GREEN + 'Formation 1' + END)
      elif running_time > formation_change_time * 6:
        print('Formation type : ' + GREEN + 'Hovering.. 1m' + END)
    
    formation.ReferenceTrajectoryGenerator(t)
    robot1_ref, robot2_ref, robot3_ref = formation.UAV_TrajectoryGenerator(running_time, formation_change_time)
    
    ros.UAV_reference_publish(N_, robot1_ref, robot2_ref, robot3_ref)
    
    if GPS_based:
      ros.UAV_GPS_based_reference_publish(N_, robot1_ref, robot2_ref, robot3_ref)
    
    past_time = int(running_time)
    loop_cnt += 1
    
    if running_time <= formation_change_time * formation_change_cnt:
      t += (nominal_speed * dt)
    else:
      t = t
      
    r.sleep()


if __name__ == '__main__':
  main()