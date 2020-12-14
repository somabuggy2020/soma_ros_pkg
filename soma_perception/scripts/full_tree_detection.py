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
                         [ 3.15, 10.4, 4],
                         [ 5.86, 8.89, 5],
                         [ 7.45, 7.28, 6],
                         [ 11.4, 4.93, 7],
                         [ 13.5, 6.91, 8],
                         [ 16.3, 5.21, 9],
                         [ 19.3, 6.18, 10],
                         [ 21.2, 3.1, 11],
                         [ 24.3, 5.53, 12],
                         [ 22.1, 9.45, 13],
                         [ 17.4, 8.78, 14],
                         [ 11.5, 8.71, 15],
                         [ 5.61, 13.3, 16],
                         [ 3.19, 15.3, 17],
                         [ 5.29, 16.9, 18],
                         [ 8.96, 15.3, 22],
                         [ 10.7, 13.1, 23],
                         [ 13.7, 11.4, 24],
                         [ 18.3, 12.9, 25],
                         [ 23.9, 13.2, 26],
                         [ 21.2, 15.6, 27],
                         [ 7.39, 19., 30],
                         [ 2.69, 22.5, 31],
                         [ 5.93, 21.3, 32],
                         [ 20.0, 19.3, 34],
                         [ 22.8, 19.8, 35],
                         [ 19.3, 22.9, 36],
                         [ 17.1, 21.7, 37],
                         [ 14.2, 23.1, 38],
                         [ 12.3, 25.1, 39],
                         [ 9.18, 24.0, 40],
                         [ 5.79, 24.4, 41],
                         [ 4.97, 29.9, 42],
                         [ 8.37, 28.6, 43],
                         [ 15.1, 26.3, 44],
                         [ 22.7, 26.7, 45]
                         ])
  return LandMarkMap


class Neural_Network():
  def __init__(self):
    self.LAND_MARKS = setLandMarks()
    self.rotate_cood()
    self.get_nearTree()
    self.make_triangle()
    self.write_csv()


  def rotate_cood(self):
    q_set = []  #list for c-space samples (x,y,theta)
    x_set = np.arange(10.0, 10.1, 1)
    y_set = np.arange(10.0, 10.1, 1)
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

  
  def write_csv(self):
    with open('/home/soma1/Documents/noboru/csv/full_tree_detection_test.csv', 'w') as file:
      writer = csv.writer(file)
      writer.writerow(['#x', 'y', 'theta',
                       'a_x', 'a_y', 
                       'b_x', 'b_y', 
                       'c_x', 'c_y',
                       'a_id', 'b_id', 'c_id'])
      for a in range(len(self.q_set)):
        writer.writerow([self.q_set[a][0], self.q_set[a][1], self.q_set[a][2],
                         self.NEARTREE[a][0][0], self.NEARTREE[a][0][1], 
                         self.NEARTREE[a][1][0], self.NEARTREE[a][1][1], 
                         self.NEARTREE[a][2][0], self.NEARTREE[a][2][1], 
                         self.NEARTREE[a][0][2], self.NEARTREE[a][1][2], self.NEARTREE[a][2][2]])

    return


if __name__ == '__main__':
  rospy.init_node('neural_network',anonymous=True)
  node = Neural_Network()