#!/usr/bin/env python

import rospy
import numpy as np
import itertools
import csv
import gc
import matplotlib.pyplot as plt

def setLandMarks():
  #36
  LandMarkMap = np.array([
                         [0.93, 1.3, 1],
                         [0.79, 4.4, 1],
                         [4.18, 4.0, 1],
                         ])
  return LandMarkMap

class Neural_Network():
  def __init__(self):
    self.LAND_MARKS = setLandMarks()
    self.rotate_cood()
    self.get_nearTree()
    self.check_dis()
    #self.write_csv()
    self.write_graph()


  def rotate_cood(self):
    q_set = []  #list for c-space samples (x,y,theta)
    x_set = np.arange(0, 5.0, 0.1)
    y_set = np.arange(2.95, 3.0, 1)
    th_set = np.arange(0, 360, 1)
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
    self.point_1 = []
    self.point_2 = []
    self.point_3 = []
    self.point_4 = []
    self.point_5 = []
    self.point_6 = []

    for a in range(len(self.q_set)):
      #123
      if self.NEARTREE[a][0][3] <= self.NEARTREE[a][1][3] and self.NEARTREE[a][1][3] <= self.NEARTREE[a][2][3]:
        self.point_1.append([self.q_set[a][0], self.q_set[a][2]])
      #132
      if self.NEARTREE[a][0][3] <= self.NEARTREE[a][2][3] and self.NEARTREE[a][2][3] <= self.NEARTREE[a][1][3]:
        self.point_2.append([self.q_set[a][0], self.q_set[a][2]])
      #213
      if self.NEARTREE[a][1][3] <= self.NEARTREE[a][0][3] and self.NEARTREE[a][0][3] <= self.NEARTREE[a][2][3]:
        self.point_3.append([self.q_set[a][0], self.q_set[a][2]])
      #231
      if self.NEARTREE[a][1][3] <= self.NEARTREE[a][2][3] and self.NEARTREE[a][2][3] <= self.NEARTREE[a][0][3]:
        self.point_4.append([self.q_set[a][0], self.q_set[a][2]])
      #312
      if self.NEARTREE[a][2][3] <= self.NEARTREE[a][0][3] and self.NEARTREE[a][0][3] <= self.NEARTREE[a][1][3]:
        self.point_5.append([self.q_set[a][0], self.q_set[a][2]])
      #321
      if self.NEARTREE[a][2][3] <= self.NEARTREE[a][1][3] and self.NEARTREE[a][1][3] <= self.NEARTREE[a][0][3]:
        self.point_6.append([self.q_set[a][0], self.q_set[a][2]])


  def write_graph(self):
    for a in range(len(self.point_1)):
      plt.scatter(self.point_1[a][0], self.point_1[a][1], c='yellow')
    for a in range(len(self.point_2)):
      plt.scatter(self.point_2[a][0], self.point_2[a][1], c='red')
    for a in range(len(self.point_3)):
      plt.scatter(self.point_3[a][0], self.point_3[a][1], c='blue')
    for a in range(len(self.point_4)):
      plt.scatter(self.point_4[a][0], self.point_4[a][1], c='green')
    for a in range(len(self.point_5)):
      plt.scatter(self.point_5[a][0], self.point_5[a][1], c='black')
    for a in range(len(self.point_6)):
      plt.scatter(self.point_6[a][0], self.point_6[a][1], c='cyan')

    plt.show()
    
    return

      

  
  def write_csv(self):
    with open('/home/soma1/Documents/noboru/csv/small_map_test.csv', 'w') as file:
      #writer = csv.writer(file)
      #writer.writerow(['123_x', '123_theta'])
      #for a in range(len(self.point)):
      #  writer.writerow([self.point[a][0], self.point[a][1]])

      writer = csv.writer(file)
      writer.writerow(['x', 'y', 'theta',
                       'tree1_x', 'tree1_y', 'tree1_d',
                       'tree2_x', 'tree2_y', 'tree2_d',
                       'tree3_x', 'tree3_y', 'tree3_d'])
      for a in range(len(self.q_set)):
        writer.writerow([self.q_set[a][0], self.q_set[a][1], self.q_set[a][2],
                         self.NEARTREE[a][0][0], self.NEARTREE[a][0][1], self.NEARTREE[a][0][3],
                         self.NEARTREE[a][1][0], self.NEARTREE[a][1][1], self.NEARTREE[a][1][3],
                         self.NEARTREE[a][2][0], self.NEARTREE[a][2][1], self.NEARTREE[a][2][3]])

    return


if __name__ == '__main__':
  rospy.init_node('neural_network',anonymous=True)
  node = Neural_Network()