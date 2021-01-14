#!/usr/bin/env python
import rospy
import math
import numpy as np
import itertools
import structure
import gc
import csv

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray


class make_treelist():
  def __init__(self):
    self.old_tree_num = 0
    rospy.Subscriber('/bounding/centroid_pose_array', PoseArray, self.call_back)
    self.list_pub = rospy.Publisher("tree_list", Float32MultiArray, queue_size=10)
    self.cood_pub = rospy.Publisher("tree_cood", Float32MultiArray, queue_size=10)
  
  def call_back(self, center):
    self.list = []

    self.tree_num = len(center.poses)

    if self.tree_num >= 3 and self.tree_num != self.old_tree_num:
      self.old_tree_num = self.tree_num
      for n in range(len(center.poses)):
        self.list.append([center.poses[n].position.y * -1, center.poses[n].position.x])
      
    if self.list != []:
      print('list : ', self.list)
      self.rotate_cood()
      self.get_nearTree()
      self.make_triangle()
      self.get_tri_vec()
      self.convert_kyoku()
      self.write_csv()
  
  def rotate_cood(self):
    q_set = []  #list for c-space samples (x,y,theta)
    x_set = np.arange(0, 1, 1)
    y_set = np.arange(0, 1, 1)
    th_set = np.arange(0, 360, 360)
    self.q_set = list(itertools.product(x_set, y_set, th_set))

    self.R_set1 = []
    self.R_set2 = []
    for x,y,th in self.q_set:
      r = np.radians(th)
      C = np.cos(r)
      S = np.sin(r)
      R_x1 = np.array([
                      [1, 0, -x],
                      [0, 1, -y],
                      [0, 0, 1]
                      ])
      R_x2 = np.array([
                      [C, -S, 0],
                      [S, C, 0],
                      [0, 0, 1]
                      ])
      self.R_set1.append(R_x1)
      self.R_set2.append(R_x2)
    return
  
  def get_nearTree(self):
    self.NEARTREE = []
    for n in range(len(self.q_set)):
      num = 0
      nearTree = []
      for m in range(len(self.list)):
        x = np.array([self.q_set[n][0], self.q_set[n][1]])
        y = np.array([self.list[m][0], self.list[m][1]])
        distance = np.linalg.norm(x - y)
        if num <= 2:
          nearTree.append(np.append(self.list[m], distance))
          nearTree.sort(key=lambda x: x[2])
        elif distance <= nearTree[2][3]:
          nearTree[2] = np.append(self.list[m], distance)
          nearTree.sort(key=lambda x: x[2])
        num += 1
      for a in range(len(nearTree)):
        LAND_MARKS_ROLL1 = np.dot(self.R_set1[n], [nearTree[a][0], nearTree[a][1], 1])
        LAND_MARKS_ROLL = np.dot(self.R_set2[n], [LAND_MARKS_ROLL1[0], LAND_MARKS_ROLL1[1], 1])
        nearTree[a] = [LAND_MARKS_ROLL[0], LAND_MARKS_ROLL[1], nearTree[a][2]]
      self.NEARTREE.append(nearTree)


  def make_triangle(self):
    for n in range(len(self.NEARTREE)):
      abD = np.sqrt((self.NEARTREE[n][0][0] - self.NEARTREE[n][1][0]) **2 + (self.NEARTREE[n][0][1] - self.NEARTREE[n][1][1]) **2)
      bcD = np.sqrt((self.NEARTREE[n][1][0] - self.NEARTREE[n][2][0]) **2 + (self.NEARTREE[n][1][1] - self.NEARTREE[n][2][1]) **2)
      caD = np.sqrt((self.NEARTREE[n][2][0] - self.NEARTREE[n][0][0]) **2 + (self.NEARTREE[n][2][1] - self.NEARTREE[n][0][1]) **2)

      if(bcD > caD):
        if(bcD < abD):
          self.NEARTREE[n][0], self.NEARTREE[n][2] = self.NEARTREE[n][2], self.NEARTREE[n][0]
      elif(caD > abD):
        self.NEARTREE[n][0], self.NEARTREE[n][1] = self.NEARTREE[n][1], self.NEARTREE[n][0]
      else:
        self.NEARTREE[n][0], self.NEARTREE[n][2] = self.NEARTREE[n][2], self.NEARTREE[n][0]

      vec_ab = np.array([(self.NEARTREE[n][1][0] - self.NEARTREE[n][0][0]), (self.NEARTREE[n][1][1] - self.NEARTREE[n][0][1])])
      vec_ac = np.array([(self.NEARTREE[n][2][0] - self.NEARTREE[n][0][0]), (self.NEARTREE[n][2][1] - self.NEARTREE[n][0][1])])
      cross = np.cross(vec_ab, vec_ac)
      if(cross > 0):
          self.NEARTREE[n][1], self.NEARTREE[n][2] = self.NEARTREE[n][2], self.NEARTREE[n][1]
    

  def get_tri_vec(self):
    self.TRI_VEC = []
    for n in range(len(self.NEARTREE)):
      ab_x = self.NEARTREE[n][0][0] - self.NEARTREE[n][1][0]
      ab_y = self.NEARTREE[n][0][1] - self.NEARTREE[n][1][1]
      bc_x = self.NEARTREE[n][1][0] - self.NEARTREE[n][2][0]
      bc_y = self.NEARTREE[n][1][1] - self.NEARTREE[n][2][1]
      ca_x = self.NEARTREE[n][2][0] - self.NEARTREE[n][0][0]
      ca_y = self.NEARTREE[n][2][1] - self.NEARTREE[n][0][1]
      self.TRI_VEC.append([ab_x, ab_y, bc_x, bc_y, ca_x, ca_y])

    for a in range(len(self.q_set)):
      tree_cood_data = Float32MultiArray()
      tree_cood_data.data = np.array([self.NEARTREE[a][0][0], self.NEARTREE[a][0][1], 
                                      self.NEARTREE[a][1][0], self.NEARTREE[a][1][1], 
                                      self.NEARTREE[a][2][0], self.NEARTREE[a][2][1]])
      self.cood_pub.publish(tree_cood_data)

  def convert_kyoku(self):
    for a in range(len(self.q_set)):
      ab_x = self.TRI_VEC[a][0]
      ab_y = self.TRI_VEC[a][1]
      bc_x = self.TRI_VEC[a][2]
      bc_y = self.TRI_VEC[a][3]
      ca_x = self.TRI_VEC[a][4]
      ca_y = self.TRI_VEC[a][5]
      self.TRI_VEC[a][0] = np.sqrt(ab_x** 2 + ab_y** 2)
      self.TRI_VEC[a][1] = np.arctan2(ab_y,ab_x)
      self.TRI_VEC[a][2] = np.sqrt(bc_x** 2 + bc_y** 2)
      self.TRI_VEC[a][3] = np.arctan2(bc_y,bc_x)
      self.TRI_VEC[a][4] = np.sqrt(ca_x** 2 + ca_y** 2)
      self.TRI_VEC[a][5] = np.arctan2(ca_y,ca_x)

      for b in range(3):
        x = self.NEARTREE[a][b][0]
        y = self.NEARTREE[a][b][1]
        self.NEARTREE[a][b][0] = np.sqrt(x** 2 + y** 2)
        self.NEARTREE[a][b][1] = np.arctan2(y,x)
  
  def write_csv(self):
    with open('/home/soma1/Documents/noboru/csv/noboru_make_treelist_test.csv', 'w') as file:
      writer = csv.writer(file)
      writer.writerow(['#x', 'y', 'theta',
                       'a_r', 'a_theta', 
                       'b_r', 'b_theta', 
                       'c_r', 'c_theta',
                       'ab_r', 'ab_theta', 'bc_r', 'bc_theta', 'ca_r', 'ca_theta'])
      for a in range(len(self.q_set)):
        writer.writerow([self.q_set[a][0], self.q_set[a][1], self.q_set[a][2],
                         self.NEARTREE[a][0][0], self.NEARTREE[a][0][1], 
                         self.NEARTREE[a][1][0], self.NEARTREE[a][1][1], 
                         self.NEARTREE[a][2][0], self.NEARTREE[a][2][1],
                         self.TRI_VEC[a][0], self.TRI_VEC[a][1], self.TRI_VEC[a][2], self.TRI_VEC[a][3], self.TRI_VEC[a][4], self.TRI_VEC[a][5]])

        tree_data = Float32MultiArray()
        tree_data.data = np.array([self.NEARTREE[a][0][0], self.NEARTREE[a][0][1], 
                              self.NEARTREE[a][1][0], self.NEARTREE[a][1][1], 
                              self.NEARTREE[a][2][0], self.NEARTREE[a][2][1],
                              self.TRI_VEC[a][0], self.TRI_VEC[a][1], self.TRI_VEC[a][2], self.TRI_VEC[a][3], self.TRI_VEC[a][4], self.TRI_VEC[a][5]])

        self.list_pub.publish(tree_data)

    return


if __name__ == '__main__':
  rospy.init_node('make_treelist', anonymous=True)
  
  node = make_treelist()

  rospy.spin()