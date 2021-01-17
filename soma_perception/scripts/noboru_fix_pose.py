#!/usr/bin/env python
import math
import numpy as np
import rospy
import rosparam
from std_msgs.msg import Float32MultiArray

from nav_msgs.msg import Odometry


class fix_pose():
  def __init__(self):
    self.icp_input = 0
    rospy.Subscriber('/odometry/filtered', Odometry, self.call_back_odom)
    rospy.Subscriber('robot_pose', Float32MultiArray, self.call_back_icp)
    self.pose_pub = rospy.Publisher("fixed_pose", Odometry, queue_size=10)

  
  def call_back_odom(self, odom_data):
    if self.icp_input == 0:

      self.odom_pose_x = odom_data.pose.pose.position.x
      self.odom_pose_y = odom_data.pose.pose.position.y

      self.pose_pub.publish(odom_data)
      #print('fix')

    else: 
      odom_data.pose.pose.position.x += self.x_dif
      odom_data.pose.pose.position.y += self.y_dif

      self.pose_pub.publish(odom_data)
      #print('fix')
      

  def call_back_icp(self, icp_data):
    self.icp_input = 1
    self.pose_x = icp_data.data[0]
    self.pose_y = icp_data.data[1]
    #self.pose_deg = icp_data.data[2]
    
    #print('pose_x', self.pose_x)
    #print('pose_y', self.pose_y)
    #print('pose_deg', self.pose_deg)

    self.x_dif = self.pose_x - self.odom_pose_x
    self.y_dif = self.pose_y - self.odom_pose_y



if __name__ == '__main__':
    rospy.init_node('fix_pose', anonymous=True)
    node = fix_pose()
    rospy.spin()