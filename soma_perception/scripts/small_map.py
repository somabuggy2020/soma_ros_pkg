#!/usr/bin/env python

import rospy
import numpy as np
import itertools
import csv
import gc

def setLandMarks():
  #36
  LandMarkMap = np.array([
                         [ 0.93, 1.3, 1],
                         [ 4.18, 4.0, 1],
                         [ 0.79, 4.4, 1],
                         ])
  return LandMarkMap

class Neural_Network():
  def __init__(self):
    self.LAND_MARKS = setLandMarks()
    self.rotate_cood()
    self.get_nearTree()
    self.check_dis()
    self.write_csv()


  def rotate_cood(self):
    q_set = []  #list for c-space samples (x,y,theta)
    x_set = np.arange(0, 5, 0.1)
    y_set = np.arange(0, 5, 0.1)
    th_set = np.arange(0, 360, 360)
    self.q_set = list(itertools.product(x_set, y_set, th_set))
    # print(x_set, y_set, th_set)
    #print(self.q_set[0])

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
      # print(R_x)
      self.R_set.append(R_x)
    
    #print('R_set:{}'.format(len(R_set)))
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
          #nearTree.sort(key=lambda x: x[3])
        elif distance <= nearTree[2][3]:
          nearTree[2] = np.append(self.LAND_MARKS[m], distance)
          #nearTree.sort(key=lambda x: x[3])
        num += 1
      for a in range(len(nearTree)):
        LAND_MARKS_ROLL = np.dot(self.R_set[n], [nearTree[a][0], nearTree[a][1], nearTree[a][2]])
        nearTree[a] = [LAND_MARKS_ROLL[0], LAND_MARKS_ROLL[1], LAND_MARKS_ROLL[2], nearTree[a][3]]
      self.NEARTREE.append(nearTree)
      
  def check_dis(self):
    self.point =[]
    for a in range(len(self.q_set)):
      if self.NEARTREE[a][2][3] <= self.NEARTREE[a][0][3] and self.NEARTREE[a][2][3] <= self.NEARTREE[a][1][3]:
        self.point.append([self.q_set[a][0], self.q_set[a][1]])
    #print('point : ', self.point)


  
  def write_csv(self):
    with open('/home/soma1/Documents/noboru/csv/small_map_graph_3.csv', 'w') as file:
      writer = csv.writer(file)
      writer.writerow(['x', 'y'])
      for a in range(len(self.point)):
        writer.writerow([self.point[a][0], self.point[a][1]])

    return

if __name__ == '__main__':
  rospy.init_node('neural_network',anonymous=True)
  node = Neural_Network()