#!/usr/bin/env python
import rospy
import math
import numpy as np
import itertools
import structure
import gc

from geometry_msgs.msg import PoseArray


class make_treelist():
  def __init__(self):
    
    self.list_2 = []
    rospy.Subscriber('/bounding_F/centroid_pose_array', PoseArray, self.call_back_F)
    rospy.Subscriber('/bounding_B/centroid_pose_array', PoseArray, self.call_back_B)
    

  def call_back_F(self, center_F):
    self.list_1 = []
    #print("0.x    ",center_F.poses[0].position.x)
    #print("0.y    ",center_F.poses[0].position.y)
    self.seq = center_F.header.seq
    for n in range(len(center_F.poses)):
      #print('num_F : ', n)
      self.list_1.append([center_F.poses[n].position.x, center_F.poses[n].position.y])

  
  def call_back_B(self, center_B):
    for m in range(len(center_B.poses)):
      #print('num_B : ', m)
      self.list_1.append([center_B.poses[m].position.x, center_B.poses[m].position.y])
    print('list : ', self.list_1)


if __name__ == '__main__':
  rospy.init_node('make_treelist', anonymous=True)
  
  node = make_treelist()

  rospy.spin()