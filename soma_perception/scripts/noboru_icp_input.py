#!/usr/bin/env python
import math
import numpy as np
import matplotlib.pyplot as plt
from icp import icp

import rospy
import rosparam
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray

def setLandMarks():
  #9
  LandMarkMap = np.array([
                         [2.3, 10.1, 1],
                         [4.2, 8.1, 2],
                         [5.1, 11.3, 3],
                         [7.1, 6.7, 4],
                         [9.3, 7.9, 5],
                         [10.0, 4.3, 6],
                         [11.2, 0.6, 7],
                         [12.8, 4.8, 8],
                         [12.7, 11.7, 9]
                         ])
  return LandMarkMap

class icp_input():
  def __init__(self):
    self.LAND_MARKS = setLandMarks()
    self.old_tree_num = 0
    self.id_num = 0
    self.cood_num = 0
    rospy.Subscriber('tree_id', Float32MultiArray, self.call_back_id)
    rospy.Subscriber('tree_cood', Float32MultiArray, self.call_back_cood)
    self.pose_pub = rospy.Publisher("robot_pose", Float32MultiArray, queue_size=10)

  
  def call_back_id(self, tree_id):
    print('tree_id : ', tree_id.data)
    self.reference_points = np.array([
                                     [self.LAND_MARKS[int(tree_id.data[0]) - 1][0], self.LAND_MARKS[int(tree_id.data[0]) - 1][1]],
                                     [self.LAND_MARKS[int(tree_id.data[1]) - 1][0], self.LAND_MARKS[int(tree_id.data[1]) - 1][1]],
                                     [self.LAND_MARKS[int(tree_id.data[2]) - 1][0], self.LAND_MARKS[int(tree_id.data[2]) - 1][1]]
                                     ])
    print('tree_cood_MAP : ', self.reference_points)
    self.id_num += 1
    self.tree_icp()

  def call_back_cood(self, cood):
    #print('tree_cood_pcd : ', cood.data)
    self.points_to_be_aligned = np.array([
                                         [cood.data[0], cood.data[1]],
                                         [cood.data[2], cood.data[3]],
                                         [cood.data[4], cood.data[5]]
                                         ])
    print('tree_cood_pcd : ', self.points_to_be_aligned)
    self.cood_num += 1
    self.tree_icp()

  
  def tree_icp(self):
    if self.id_num == self.cood_num and self.id_num >= 1:

      robot_pose = [0, 0, 0]

      robot_pose[0] = self.reference_points[0][0] - self.points_to_be_aligned[0][0]
      robot_pose[1] = self.reference_points[0][1] - self.points_to_be_aligned[0][1]

    for n in range(len(self.points_to_be_aligned)):
      self.points_to_be_aligned[n][0] += robot_pose[0]
      self.points_to_be_aligned[n][1] += robot_pose[1]
    

    print('robot pose : ', robot_pose)


    # run icp
    #transformation_history, aligned_points = icp(reference_points, points_to_be_aligned, verbose=True)
    transformation_history, aligned_points, robot_pose_cor = icp(self.reference_points, self.points_to_be_aligned, robot_pose, verbose=True)

    print('robot pose_cor : ', robot_pose_cor)
    pose = Float32MultiArray()
    pose.data = np.array([robot_pose_cor[0], robot_pose_cor[1], robot_pose_cor[2]])
    if robot_pose != robot_pose_cor:
      self.pose_pub.publish(pose)
    else:
      print('icp_error')

    # show results
    plt.plot(self.reference_points[:, 0], self.reference_points[:, 1], 'rx', label='reference points')
    plt.plot(self.points_to_be_aligned[:, 0], self.points_to_be_aligned[:, 1], 'b1', label='points to be aligned')
    plt.plot(aligned_points[:, 0], aligned_points[:, 1], 'g+', label='aligned points')
    plt.plot(robot_pose[0], robot_pose[1], 'b^', label='robot pose')
    plt.plot(robot_pose_cor[0], robot_pose_cor[1], 'g^', label='robot pose ICP')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    rospy.init_node('icp', anonymous=True)
    node = icp_input()
    rospy.spin()