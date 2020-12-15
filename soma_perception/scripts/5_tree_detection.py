#!/usr/bin/env python

import rospy
import numpy as np
import itertools
import csv
import gc
import matplotlib.pyplot as plt

def setLandMarks():
  #4
  LandMarkMap = np.array([
                         [2.3, 5.1, 1],
                         [4.2, 3.1, 2],
                         [5.1, 6.3, 3],
                         [7.1, 1.7, 4],
                         [9.3, 2.9, 5]
                         ])
  return LandMarkMap


class Neural_Network():
  def __init__(self):
    self.LAND_MARKS = setLandMarks()
    self.rotate_cood()
    self.get_nearTree()
    self.make_triangle()
    self.probability()
    self.write_csv()


  def rotate_cood(self):
    q_set = []  #list for c-space samples (x,y,theta)
    x_set = np.arange(2, 9, 0.05)
    y_set = np.arange(2, 8, 0.05)
    th_set = np.arange(0, 360, 360)
    self.q_set = list(itertools.product(x_set, y_set, th_set))


    self.R_set = []
    for x,y,th in self.q_set:
      r = np.radians(th)
      C = np.cos(r)
      S = np.sin(r)
      R_x = np.array([
                      [C, -S, -x],
                      [S, C, -y],
                      [0, 0, 1]
                      ])
      self.R_set.append(R_x)
    
    return 

  def get_nearTree(self):
    self.NEARTREE = []
    for n in range(len(self.q_set)):
      num = 0
      nearTree = []
      for m in range(len(self.LAND_MARKS)):
        x = np.array([self.q_set[n][0], self.q_set[n][1]])
        y = np.array([self.LAND_MARKS[m][0], self.LAND_MARKS[m][1]])
        distance = np.linalg.norm(x - y)
        if num <= 2:
          nearTree.append(np.append(self.LAND_MARKS[m], distance))
          nearTree.sort(key=lambda x: x[3])
        elif distance <= nearTree[2][3]:
          nearTree[2] = np.append(self.LAND_MARKS[m], distance)
          nearTree.sort(key=lambda x: x[3])
        num += 1
      for a in range(len(nearTree)):
        LAND_MARKS_ROLL = np.dot(self.R_set[n], [nearTree[a][0], nearTree[a][1], 1])
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


  def probability(self):
    self.PROB =[]
    for n in range(len(self.NEARTREE)):
      prob = []
      for m in range(3):
        ID = self.NEARTREE[n][m][2]
        prob.append([0, 0, 0, 0, 0])
        prob[m][int(ID)-1] = 1
      self.PROB.append(prob)

  
  def write_csv(self):
    with open('/home/soma1/Documents/noboru/csv/5_tree_detection_test.csv', 'w') as file:
      writer = csv.writer(file)
      writer.writerow(['#x', 'y', 'theta',
                       'a_x', 'a_y', 
                       'b_x', 'b_y', 
                       'c_x', 'c_y', 
                       'a_id', 'b_id', 'c_id', 
                       'p_a1', 'p_a2', 'p_a3', 'p_a4', 'p_a5',
                       'p_b1', 'p_b2', 'p_b3', 'p_b4', 'p_b5',
                       'p_c1', 'p_c2', 'p_c3', 'p_c4', 'p_c5'])
      for a in range(len(self.q_set)):
        writer.writerow([self.q_set[a][0], self.q_set[a][1], self.q_set[a][2],
                         self.NEARTREE[a][0][0], self.NEARTREE[a][0][1], 
                         self.NEARTREE[a][1][0], self.NEARTREE[a][1][1], 
                         self.NEARTREE[a][2][0], self.NEARTREE[a][2][1], 
                         self.NEARTREE[a][0][2], self.NEARTREE[a][1][2], self.NEARTREE[a][2][2],
                         self.PROB[a][0][0], self.PROB[a][0][1], self.PROB[a][0][2], self.PROB[a][0][3], self.PROB[a][0][4],
                         self.PROB[a][1][0], self.PROB[a][1][1], self.PROB[a][1][2], self.PROB[a][1][3], self.PROB[a][1][4],
                         self.PROB[a][2][0], self.PROB[a][2][1], self.PROB[a][2][2], self.PROB[a][2][3], self.PROB[a][2][4]])

    return


if __name__ == '__main__':
  rospy.init_node('neural_network',anonymous=True)
  node = Neural_Network()