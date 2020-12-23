#!/usr/bin/env python

import rospy
import numpy as np
import itertools
import csv
import gc
import matplotlib.pyplot as plt

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


class Neural_Network():
  def __init__(self):
    self.LAND_MARKS = setLandMarks()
    self.rotate_cood()
    self.get_nearTree()
    self.make_triangle()
    self.get_tri_vec()
    self.convert_kyoku()
    self.probability()
    self.write_csv()


  def rotate_cood(self):
    q_set = []  #list for c-space samples (x,y,theta)
    x_set = np.arange(0.05, 15.05, 0.1)
    y_set = np.arange(0.05, 15.05, 0.1)
    th_set = np.arange(330, 360, 1)
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
      tree_dis = []
      nearTree = []

      for m in range(len(self.LAND_MARKS)):
        x = np.array([self.q_set[n][0], self.q_set[n][1]])
        y = np.array([self.LAND_MARKS[m][0], self.LAND_MARKS[m][1]])
        distance = np.linalg.norm(x - y)
        tree_dis.append(np.append(self.LAND_MARKS[m], distance))
        tree_dis.sort(key=lambda x: x[3])

      for a in range(len(tree_dis)):
        LAND_MARKS_ROLL1 = np.dot(self.R_set1[n], [tree_dis[a][0], tree_dis[a][1], 1])
        LAND_MARKS_ROLL = np.dot(self.R_set2[n], [LAND_MARKS_ROLL1[0], LAND_MARKS_ROLL1[1], 1])
        dig = np.arctan2(LAND_MARKS_ROLL[1], LAND_MARKS_ROLL[0])

        if (tree_dis[a][3] <= 6 and (np.radians(55.5) < dig < np.radians(124.5) or np.radians(-124.5) < dig < np.radians(-55.5))):
          if num >= 3:
            break
          nearTree.append([LAND_MARKS_ROLL[0], LAND_MARKS_ROLL[1], tree_dis[a][2], self.q_set[n][0], self.q_set[n][1], self.q_set[n][2]])
          num += 1

      if len(nearTree) == 3:
        self.NEARTREE.append(nearTree)


  def make_triangle(self):
    for n in range(len(self.NEARTREE)):
      #abD = np.sqrt((self.NEARTREE[n][0][0] - self.NEARTREE[n][1][0]) **2 + (self.NEARTREE[n][0][1] - self.NEARTREE[n][1][1]) **2)
      #bcD = np.sqrt((self.NEARTREE[n][1][0] - self.NEARTREE[n][2][0]) **2 + (self.NEARTREE[n][1][1] - self.NEARTREE[n][2][1]) **2)
      #caD = np.sqrt((self.NEARTREE[n][2][0] - self.NEARTREE[n][0][0]) **2 + (self.NEARTREE[n][2][1] - self.NEARTREE[n][0][1]) **2)
#
      #if(bcD > caD):
      #  if(bcD < abD):
      #    self.NEARTREE[n][0], self.NEARTREE[n][2] = self.NEARTREE[n][2], self.NEARTREE[n][0]
      #elif(caD > abD):
      #  self.NEARTREE[n][0], self.NEARTREE[n][1] = self.NEARTREE[n][1], self.NEARTREE[n][0]
      #else:
      #  self.NEARTREE[n][0], self.NEARTREE[n][2] = self.NEARTREE[n][2], self.NEARTREE[n][0]

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

  def convert_kyoku(self):
    for a in range(len(self.NEARTREE)):
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


  def probability(self):
    self.PROB =[]
    for n in range(len(self.NEARTREE)):
      prob = []
      for m in range(3):
        ID = self.NEARTREE[n][m][2]
        prob.append([0, 0, 0, 0, 0, 0, 0, 0, 0])
        prob[m][int(ID)-1] = 1
      self.PROB.append(prob)

  
  def write_csv(self):
    with open('/home/soma1/Documents/noboru/csv/trivec_kyoku_NEWrange_9_tree_detection_test_test.csv', 'a') as file:
      writer = csv.writer(file)
      #writer.writerow(['#x', 'y', 'theta',
      #                 'a_r', 'a_theta', 
      #                 'b_r', 'b_theta', 
      #                 'c_r', 'c_theta',
      #                 'ab_r', 'ab_theta', 'bc_r', 'bc_theta', 'ca_r', 'ca_theta', 
      #                 'a_id', 'b_id', 'c_id', 
      #                 'p_a1', 'p_a2', 'p_a3', 'p_a4', 'p_a5', 'p_a6', 'p_a7', 'p_a8', 'p_a9',
      #                 'p_b1', 'p_b2', 'p_b3', 'p_b4', 'p_b5', 'p_b6', 'p_b7', 'p_b8', 'p_b9',
      #                 'p_c1', 'p_c2', 'p_c3', 'p_c4', 'p_c5', 'p_c6', 'p_c7', 'p_c8', 'p_c9'])

      for a in range(len(self.NEARTREE)):
        writer.writerow([self.NEARTREE[a][0][3], self.NEARTREE[a][0][4], self.NEARTREE[a][0][5],
                         self.NEARTREE[a][0][0], self.NEARTREE[a][0][1], 
                         self.NEARTREE[a][1][0], self.NEARTREE[a][1][1], 
                         self.NEARTREE[a][2][0], self.NEARTREE[a][2][1],
                         self.TRI_VEC[a][0], self.TRI_VEC[a][1], self.TRI_VEC[a][2], self.TRI_VEC[a][3], self.TRI_VEC[a][4], self.TRI_VEC[a][5], 
                         self.NEARTREE[a][0][2], self.NEARTREE[a][1][2], self.NEARTREE[a][2][2],
                         self.PROB[a][0][0], self.PROB[a][0][1], self.PROB[a][0][2], self.PROB[a][0][3], self.PROB[a][0][4], self.PROB[a][0][5], self.PROB[a][0][6], self.PROB[a][0][7], self.PROB[a][0][8],
                         self.PROB[a][1][0], self.PROB[a][1][1], self.PROB[a][1][2], self.PROB[a][1][3], self.PROB[a][1][4], self.PROB[a][1][5], self.PROB[a][1][6], self.PROB[a][1][7], self.PROB[a][1][8],
                         self.PROB[a][2][0], self.PROB[a][2][1], self.PROB[a][2][2], self.PROB[a][2][3], self.PROB[a][2][4], self.PROB[a][2][5], self.PROB[a][2][6], self.PROB[a][2][7], self.PROB[a][2][8]])

    return


if __name__ == '__main__':
  rospy.init_node('neural_network',anonymous=True)
  node = Neural_Network()