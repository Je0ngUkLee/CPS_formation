#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

CPS Formation Maker

author: Jeong-Uk Lee (@sch.ac.kr)


"""

import sys
import math
import rospy
import timeit
import signal

import numpy as np

from mavros_msgs.msg import State
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker

from multiprocessing import Queue
from tf.transformations import euler_from_quaternion


def signal_handler(signal, frame):
  print('\nYou pressed Ctrl+C!')
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class ROS():
  
  def __init__(self):
    rospy.init_node('CPS_Formation_Maker')
    
    self.leader_state = ''
    self.robot2_state = ''
    self.robot3_state = ''
    
    self.leader_pos = PoseStamped()  ; self.robot2_pos = PoseStamped()  ; self.robot3_pos = PoseStamped()
    self.leader_pos_queue = Queue()  ; self.robot2_pos_queue = Queue()  ; self.robot3_pos_queue = Queue()
    self.leader_state_queue = Queue(); self.robot2_state_queue = Queue(); self.robot3_state_queue = Queue()
    
    
    self.leader_pos_sub = rospy.Subscriber('UAV1/mavros/local_position/pose', PoseStamped, self.pos_callback, queue_size = 1)
    self.leader_state_sub = rospy.Subscriber('UAV1/mavros/state', State, self.state_callback, queue_size = 1)
    
    self.robot2_pos_sub = rospy.Subscriber('UAV2/mavros/local_position/pose', PoseStamped, self.pos2_callback, queue_size = 1)
    self.robot2_state_sub = rospy.Subscriber('UAV2/mavros/state', State, self.state2_callback, queue_size = 1)
    
    self.robot3_pos_sub = rospy.Subscriber('UAV3/mavros/local_position/pose', PoseStamped, self.pos3_callback, queue_size = 1)
    self.robot3_state_sub = rospy.Subscriber('UAV3/mavros/state', State, self.state3_callback, queue_size = 1)
    
    
    self.rbt1_pose_pub = rospy.Publisher('UAV1/current_position', Marker, queue_size = 1)
    self.rbt2_pose_pub = rospy.Publisher('UAV2/current_position', Marker, queue_size = 1)
    self.rbt3_pose_pub = rospy.Publisher('UAV3/current_position', Marker, queue_size = 1)
    
    
    self.rbt1_setpoint_pub     = rospy.Publisher('UAV1/mavros/setpoint_position/local', PoseStamped, queue_size = 1)
    self.rbt1_visual_point_pub = rospy.Publisher('UAV1/target_setpoint', Marker, queue_size = 1)
    
    self.rbt2_setpoint_pub = rospy.Publisher('UAV2/mavros/setpoint_position/local', PoseStamped, queue_size = 1)
    self.rbt2_visual_point_pub = rospy.Publisher('UAV2/target_setpoint', Marker, queue_size = 1)
    
    self.rbt3_setpoint_pub = rospy.Publisher('UAV3/mavros/setpoint_position/local', PoseStamped, queue_size = 1)
    self.rbt3_visual_point_pub = rospy.Publisher('UAV3/target_setpoint', Marker, queue_size = 1)
    
  
  def pos_callback(self, msg):
    self.leader_pos_queue.put(msg)
    self.leader_pos = msg
    
    self.leader_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    
  def pos2_callback(self, msg):
    self.robot2_pos_queue.put(msg)
    self.robot2_pos = msg
    
    self.robot2_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    
  def pos3_callback(self, msg):
    self.robot3_pos_queue.put(msg)
    self.robot3_pos = msg
    
    self.robot3_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
  
    
  def state_callback(self, msg):
    self.leader_state_queue.put(msg)
    self.leader_state = msg.mode
    
  def state2_callback(self, msg):
    self.robot2_state_queue.put(msg)
    self.robot2_state = msg.mode
    
  def state3_callback(self, msg):
    self.robot3_state_queue.put(msg)
    self.robot3_state = msg.mode
  
    
  
  def rbt1_curr_pose_publish(self):
    pub_msg = Marker()
    
    pub_msg.header.frame_id = 'map'
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.ns = 'points'
    pub_msg.id = 1
    pub_msg.type = Marker.POINTS
    pub_msg.action = Marker.ADD
    pub_msg.color = ColorRGBA(1, 0, 0, 0.5)
    pub_msg.scale.x = 0.6
    pub_msg.scale.y = 0.6
    pub_msg.scale.z = 0.6
    pub_msg.points.append(Point(self.leader_position[0], self.leader_position[1], self.leader_position[2]))
    
    self.rbt1_pose_pub.publish(pub_msg)
    
  def rbt2_curr_pose_publish(self):
    pub_msg = Marker()
    
    pub_msg.header.frame_id = 'map'
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.ns = 'points'
    pub_msg.id = 2
    pub_msg.type = Marker.POINTS
    pub_msg.action = Marker.ADD
    pub_msg.color = ColorRGBA(1, 0, 0, 0.5)
    pub_msg.scale.x = 0.6
    pub_msg.scale.y = 0.6
    pub_msg.scale.z = 0.6
    pub_msg.points.append(Point(self.robot2_position[0], self.robot2_position[1] - 3, self.robot2_position[2]))
    
    self.rbt2_pose_pub.publish(pub_msg)
    
  def rbt3_curr_pose_publish(self):
    pub_msg = Marker()
    
    pub_msg.header.frame_id = 'map'
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.ns = 'points'
    pub_msg.id = 3
    pub_msg.type = Marker.POINTS
    pub_msg.action = Marker.ADD
    pub_msg.color = ColorRGBA(1, 0, 0, 0.5)
    pub_msg.scale.x = 0.6
    pub_msg.scale.y = 0.6
    pub_msg.scale.z = 0.6
    pub_msg.points.append(Point(self.robot3_position[0], self.robot3_position[1], self.robot3_position[2]))
    
    self.rbt3_pose_pub.publish(pub_msg)
    
    
  def rbt1_setpoint_publish(self, point):
    setpoint = PoseStamped()
    
    setpoint.header.frame_id = 'map'
    setpoint.header.stamp = rospy.Time.now()
    setpoint.pose.position.x = point[0] + self.pos1_mean[0]
    setpoint.pose.position.y = point[1] + self.pos1_mean[1]
    setpoint.pose.position.z = point[2] + self.pos1_mean[2]
    setpoint.pose.orientation.x = self.pos1_mean[3]
    setpoint.pose.orientation.y = self.pos1_mean[4]
    setpoint.pose.orientation.z = self.pos1_mean[5]
    setpoint.pose.orientation.w = self.pos1_mean[6]
    self.rbt1_setpoint_pub.publish(setpoint)
    
    
    visual_point = Marker()
    
    visual_point.header.frame_id = 'map'
    visual_point.header.stamp = rospy.Time.now()
    visual_point.ns = 'points'
    visual_point.id = 4
    visual_point.type = Marker.POINTS
    visual_point.action = Marker.ADD
    visual_point.color = ColorRGBA(0, 0, 1, 0.5)
    visual_point.scale.x = 0.6
    visual_point.scale.y = 0.6
    visual_point.scale.z = 0.6
    visual_point.points.append(Point(point[0], point[1], point[2]))
    self.rbt1_visual_point_pub.publish(visual_point)
    
  def rbt2_setpoint_publish(self, point):
    setpoint = PoseStamped()
    
    setpoint.header.frame_id = 'map'
    setpoint.header.stamp = rospy.Time.now()
    setpoint.pose.position.x = point[0]       + self.pos2_mean[0]
    setpoint.pose.position.y = point[1] + 3.0 + self.pos2_mean[1]
    setpoint.pose.position.z = point[2]       + self.pos2_mean[2]
    setpoint.pose.orientation.x = self.pos2_mean[3]
    setpoint.pose.orientation.y = self.pos2_mean[4]
    setpoint.pose.orientation.z = self.pos2_mean[5]
    setpoint.pose.orientation.w = self.pos2_mean[6]
    self.rbt2_setpoint_pub.publish(setpoint)
    
    
    visual_point = Marker()
    
    visual_point.header.frame_id = 'map'
    visual_point.header.stamp = rospy.Time.now()
    visual_point.ns = 'points'
    visual_point.id = 5
    visual_point.type = Marker.POINTS
    visual_point.action = Marker.ADD
    visual_point.color = ColorRGBA(0, 0, 1, 0.5)
    visual_point.scale.x = 0.6
    visual_point.scale.y = 0.6
    visual_point.scale.z = 0.6
    visual_point.points.append(Point(point[0], point[1], point[2]))
    self.rbt2_visual_point_pub.publish(visual_point)
    
  def rbt3_setpoint_publish(self, point):
    setpoint = PoseStamped()
    
    setpoint.header.frame_id = 'map'
    setpoint.header.stamp = rospy.Time.now()
    setpoint.pose.position.x = point[0]       + self.pos3_mean[0]
    setpoint.pose.position.y = point[1] - 3.0 + self.pos3_mean[1]
    setpoint.pose.position.z = point[2]       + self.pos3_mean[2]
    setpoint.pose.orientation.x = self.pos3_mean[3]
    setpoint.pose.orientation.y = self.pos3_mean[4]
    setpoint.pose.orientation.z = self.pos3_mean[5]
    setpoint.pose.orientation.w = self.pos3_mean[6]
    self.rbt3_setpoint_pub.publish(setpoint)
    
    
    visual_point = Marker()
    
    visual_point.header.frame_id = 'map'
    visual_point.header.stamp = rospy.Time.now()
    visual_point.ns = 'points'
    visual_point.id = 6
    visual_point.type = Marker.POINTS
    visual_point.action = Marker.ADD
    visual_point.color = ColorRGBA(0, 0, 1, 0.5)
    visual_point.scale.x = 0.6
    visual_point.scale.y = 0.6
    visual_point.scale.z = 0.6
    visual_point.points.append(Point(point[0], point[1], point[2]))
    self.rbt3_visual_point_pub.publish(visual_point)
    
    
    
    
  def averaging_poses(self):
    pos1_history = []
    pos2_history = []
    pos3_history = []
    self.pos1_mean = np.zeros(7)
    self.pos2_mean = np.zeros(7)
    self.pos3_mean = np.zeros(7)
    
    while 1:
      pos1_history.append([self.leader_pos.pose.position.x, self.leader_pos.pose.position.y, self.leader_pos.pose.position.z, \
                          self.leader_pos.pose.orientation.x, self.leader_pos.pose.orientation.y, self.leader_pos.pose.orientation.z, self.leader_pos.pose.orientation.w])
      
      pos2_history.append([self.robot2_pos.pose.position.x, self.robot2_pos.pose.position.y, self.robot2_pos.pose.position.z, \
                          self.robot2_pos.pose.orientation.x, self.robot2_pos.pose.orientation.y, self.robot2_pos.pose.orientation.z, self.robot2_pos.pose.orientation.w])
      
      pos3_history.append([self.robot3_pos.pose.position.x, self.robot3_pos.pose.position.y, self.robot3_pos.pose.position.z, \
                          self.robot3_pos.pose.orientation.x, self.robot3_pos.pose.orientation.y, self.robot3_pos.pose.orientation.z, self.robot3_pos.pose.orientation.w])
      
      r = rospy.Rate(10)
      if len(pos1_history) < 51:
        print('Averaging_poses')
        r.sleep()
        continue
      else:
        mean_x1 = 0.0; mean_y1 = 0.0; mean_z1 = 0.0; mean_ori_x1 = 0.0; mean_ori_y1 = 0.0; mean_ori_z1 = 0.0; mean_ori_w1 = 0.0
        mean_x2 = 0.0; mean_y2 = 0.0; mean_z2 = 0.0; mean_ori_x2 = 0.0; mean_ori_y2 = 0.0; mean_ori_z2 = 0.0; mean_ori_w2 = 0.0
        mean_x3 = 0.0; mean_y3 = 0.0; mean_z3 = 0.0; mean_ori_x3 = 0.0; mean_ori_y3 = 0.0; mean_ori_z3 = 0.0; mean_ori_w3 = 0.0
        
        for i in range(len(pos1_history)):
          mean_x1     += pos1_history[i][0]
          mean_y1     += pos1_history[i][1]
          mean_z1     += pos1_history[i][2]
          mean_ori_x1 += pos1_history[i][3]
          mean_ori_y1 += pos1_history[i][4]
          mean_ori_z1 += pos1_history[i][5]
          mean_ori_w1 += pos1_history[i][6]
          
          mean_x2     += pos2_history[i][0]
          mean_y2     += pos2_history[i][1]
          mean_z2     += pos2_history[i][2]
          mean_ori_x2 += pos2_history[i][3]
          mean_ori_y2 += pos2_history[i][4]
          mean_ori_z2 += pos2_history[i][5]
          mean_ori_w2 += pos2_history[i][6]
          
          mean_x3     += pos3_history[i][0]
          mean_y3     += pos3_history[i][1]
          mean_z3     += pos3_history[i][2]
          mean_ori_x3 += pos3_history[i][3]
          mean_ori_y3 += pos3_history[i][4]
          mean_ori_z3 += pos3_history[i][5]
          mean_ori_w3 += pos3_history[i][6]

        mean_x1     = mean_x1 / len(pos1_history)
        mean_y1     = mean_y1 / len(pos1_history)
        mean_z1     = mean_z1 / len(pos1_history)
        mean_ori_x1 = mean_ori_x1 / len(pos1_history)
        mean_ori_y1 = mean_ori_y1 / len(pos1_history)
        mean_ori_z1 = mean_ori_z1 / len(pos1_history)
        mean_ori_w1 = mean_ori_w1 / len(pos1_history)
        
        mean_x2     = mean_x2 / len(pos2_history)
        mean_y2     = mean_y2 / len(pos2_history)
        mean_z2     = mean_z2 / len(pos2_history)
        mean_ori_x2 = mean_ori_x2 / len(pos2_history)
        mean_ori_y2 = mean_ori_y2 / len(pos2_history)
        mean_ori_z2 = mean_ori_z2 / len(pos2_history)
        mean_ori_w2 = mean_ori_w2 / len(pos2_history)
        
        mean_x3     = mean_x3 / len(pos3_history)
        mean_y3     = mean_y3 / len(pos3_history)
        mean_z3     = mean_z3 / len(pos3_history)
        mean_ori_x3 = mean_ori_x3 / len(pos3_history)
        mean_ori_y3 = mean_ori_y3 / len(pos3_history)
        mean_ori_z3 = mean_ori_z3 / len(pos3_history)
        mean_ori_w3 = mean_ori_w3 / len(pos3_history)


        self.pos1_mean[0] = mean_x1
        self.pos1_mean[1] = mean_y1
        self.pos1_mean[2] = mean_z1
        self.pos1_mean[3] = mean_ori_x1
        self.pos1_mean[4] = mean_ori_y1
        self.pos1_mean[5] = mean_ori_z1
        self.pos1_mean[6] = mean_ori_w1
        
        self.pos2_mean[0] = mean_x2
        self.pos2_mean[1] = mean_y2
        self.pos2_mean[2] = mean_z2
        self.pos2_mean[3] = mean_ori_x2
        self.pos2_mean[4] = mean_ori_y2
        self.pos2_mean[5] = mean_ori_z2
        self.pos2_mean[6] = mean_ori_w2
        
        self.pos3_mean[0] = mean_x3
        self.pos3_mean[1] = mean_y3
        self.pos3_mean[2] = mean_z3
        self.pos3_mean[3] = mean_ori_x3
        self.pos3_mean[4] = mean_ori_y3
        self.pos3_mean[5] = mean_ori_z3
        self.pos3_mean[6] = mean_ori_w3
      
      
        print('Averaging_poses is completed!')
        break
  
  
  def rotation(self, point):
    
    result = np.zeros(3)
    
    roll_1, pitch_1, yaw = euler_from_quaternion((self.pos1_mean[3], self.pos1_mean[4], self.pos1_mean[5], self.pos1_mean[6]))

    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                                [np.sin(yaw), np.cos(yaw)]])
    
    trans_x = point[0] - self.pos1_mean[0]
    trans_y = point[1] - self.pos1_mean[1]
    
    trans_ = (trans_x, trans_y)
    
    rotated_point = np.dot(rotation_matrix, trans_) + (self.pos1_mean[0], self.pos1_mean[1])
    
    result[0] = rotated_point[0]
    result[1] = rotated_point[1]
    result[2] = point[2]
    
    return result
  
        
        
    
class Formation():
  
  def __init__(self):
    
    self.uav1_pos_list = [[0.0, 0.0, 2.0]]; self.uav2_pos_list = [[0.0, -3.0, 2.0]]; self.uav3_pos_list = [[0.0, 3.0, 2.0]]
    
    formation_interval = 5 # [m]
    interval = 0.5         # [m]
    
    for i in range(int(formation_interval / interval)):
      self.uav1_pos_list.append([i * interval + interval,  0.0, 2.0])
      self.uav2_pos_list.append([i * interval + interval, -3.0, 2.0])
      self.uav3_pos_list.append([i * interval + interval,  3.0, 2.0])
    x_end = self.uav1_pos_list[-1][0] + interval
      
    for i in range(int(formation_interval / interval)):
      self.uav1_pos_list.append([i * interval + x_end      ,  0.0, 2.0])
      self.uav2_pos_list.append([i * interval + x_end - 1.5, -3.0, 2.0])
      self.uav3_pos_list.append([i * interval + x_end - 1.5,  3.0, 2.0])
    x_end = self.uav1_pos_list[-1][0] + interval
      
    for i in range(int(formation_interval / interval)):
      self.uav1_pos_list.append([i * interval + x_end,  0.0, 2.0])
      self.uav2_pos_list.append([i * interval + x_end, -3.0, 2.0])
      self.uav3_pos_list.append([i * interval + x_end,  3.0, 2.0])
    x_end = self.uav1_pos_list[-1][0] + interval
      
    for i in range(int(formation_interval / interval)):
      self.uav1_pos_list.append([i * interval + x_end,  0.0, 2.0])
      self.uav2_pos_list.append([i * interval + x_end, -3.0, 3.0])
      self.uav3_pos_list.append([i * interval + x_end,  3.0, 3.0])
    x_end = self.uav1_pos_list[-1][0] + interval
      
    for i in range(int(formation_interval / interval)):
      self.uav1_pos_list.append([i * interval + x_end,  0.0, 2.0])
      self.uav2_pos_list.append([i * interval + x_end, -3.0, 2.0])
      self.uav3_pos_list.append([i * interval + x_end,  3.0, 2.0])
    x_end = self.uav1_pos_list[-1][0] + interval
    
    for i in range(10):
      self.uav1_pos_list.append([x_end,  0.0, 2.0 - i * 0.1])
      self.uav2_pos_list.append([x_end, -3.0, 2.0 - i * 0.1])
      self.uav3_pos_list.append([x_end,  3.0, 2.0 - i * 0.1])
    
    # for i in range(int(formation_interval / interval)):
    #   self.uav1_pos_list.append([i * interval + x_end,  0.0, 1.0])
    #   self.uav2_pos_list.append([i * interval + x_end, -3.0, 1.0])
    #   self.uav3_pos_list.append([i * interval + x_end,  3.0, 1.0])
    # x_end = self.uav1_pos_list[-1][0] + interval
      
      
    
    
def main():
  print(__file__ + " Start !!")
  
  ros = ROS()
  formation = Formation()
  
  Robot1 = True
  Robot2 = True
  Robot3 = False
  
  while 1:
    if ros.leader_state_queue.empty() and Robot1:
      print('Leader state is not subscribed!')
      continue
    elif ros.robot2_state_queue.empty() and Robot2:
      print('Robot2 state is not subscribed!')
      continue
    elif ros.robot3_state_queue.empty() and Robot3:
      print('Robot3 state is not subscribed!')
      continue
    else: break
  
  while 1:
    if ros.leader_pos_queue.empty() and Robot1: 
      print('Leader local pose is not subscribed!')
      continue
    elif ros.robot2_pos_queue.empty() and Robot2:
      print('Robot2 state is not subscribed!')
      continue
    elif ros.robot3_pos_queue.empty() and Robot3:
      print('Robot3 state is not subscribed!')
      continue
    else: break
    
  # while 1:
  #   if ros.leader_state != 'OFFBOARD' and Robot1:
  #     print('Leader current state is not OFFBOARD')
  #     continue
  #   elif ros.robot2_state != 'OFFBOARD' and Robot2:
  #     print('Robot2 current state is not OFFBOARD')
  #     continue
  #   elif ros.robot3_state != 'OFFBOARD' and Robot3:
  #     print('Robot3 current state is not OFFBOARD')
  #     continue
  #   else: break
  
  ros.averaging_poses()
  
  pos_index = 0
  r = rospy.Rate(5)
  
  start_time = timeit.default_timer()
  
  while not rospy.is_shutdown():
    if Robot1:
      ros.rbt1_curr_pose_publish()
    if Robot2:
      ros.rbt2_curr_pose_publish()
    if Robot3:
      ros.rbt3_curr_pose_publish()
      
    running_time = timeit.default_timer() - start_time
    
    cur_leader_pos = ros.leader_position
    
    if running_time < 5:  # 5초 동안 hovering
      print('Hovering..')
      pos_index = 0
    
    uav1_set_point = formation.uav1_pos_list[pos_index]
    uav2_set_point = formation.uav2_pos_list[pos_index]
    uav3_set_point = formation.uav3_pos_list[pos_index]
    
    rotated_uav1_set_point = ros.rotation(uav1_set_point)
    rotated_uav2_set_point = ros.rotation(uav2_set_point)
    rotated_uav3_set_point = ros.rotation(uav3_set_point)
    
    if Robot1:
      ros.rbt1_setpoint_publish(rotated_uav1_set_point)
    if Robot2:
      ros.rbt2_setpoint_publish(rotated_uav2_set_point)
    if Robot3:
      ros.rbt3_setpoint_publish(rotated_uav3_set_point)
    
    # if running_time - start_time < 5:  # 5초 동안 hovering
    #   print('Hovering..')
    #   continue
    
    leader_2dim_distance = math.hypot(rotated_uav1_set_point[0] - cur_leader_pos[0], rotated_uav1_set_point[1] - cur_leader_pos[1])
    
    if pos_index < 11 and running_time > 5:
      print('Formation 1')
    elif pos_index >= 11 and pos_index < 20:
      print('Formation 2')
    elif pos_index >= 21 and pos_index < 30:
      print('Formation 1')
    elif pos_index >= 31 and pos_index < 40:
      print('Formation 3')
    elif pos_index >= 41 and pos_index < 50:
      print('Formation 1')
    elif pos_index >= 51 and pos_index < 60:
      print('Formation 1_under')
    elif pos_index >= 60:
      print('Finish!')
    
    # print('Distance: {}[m]'.format(leader_2dim_distance))
    
    # print('Leader target pose', uav1_set_point)
    # print('Robot2 target pose', uav2_set_point)
    # print('Robot3 target pose', uav3_set_point)
    
    # if leader_2dim_distance < 0.3 and pos_index < 60:
    if leader_2dim_distance < 5.0 and pos_index < 60:
      pos_index += 1

    r.sleep()
  
  
  
if __name__ == '__main__':
  main()