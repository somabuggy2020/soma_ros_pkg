#!/usr/bin/env python

import rospy
import numpy as np
import itertools
import csv
import gc

def setLandMarks():
  #36
  LandMarkMap = np.array([
                         [ 3.15, 10.4, 1],
                         [ 5.86, 8.89, 1],
                         [ 7.45, 7.28, 1],
                         [ 11.4, 4.93, 1],
                         [ 13.5, 6.91, 1],
                         [ 16.3, 5.21, 1],
                         [ 19.3, 6.18, 1],
                         [ 21.2, 3.1, 1],
                         [ 24.3, 5.53, 1],
                         [ 22.1, 9.45, 1],
                         [ 17.4, 8.78, 1],
                         [ 11.5, 8.71, 1],
                         [ 5.61, 13.3, 1],
                         [ 3.19, 15.3, 1],
                         [ 5.29, 16.9, 1],
                         [ 8.96, 15.3, 1],
                         [ 10.7, 13.1, 1],
                         [ 13.7, 11.4, 1],
                         [ 18.3, 12.9, 1],
                         [ 23.9, 13.2, 1],
                         [ 21.2, 15.6, 1],
                         [ 7.39, 19., 1],
                         [ 2.69, 22.5, 1],
                         [ 5.93, 21.3, 1],
                         [ 20.0, 19.3, 1],
                         [ 22.8, 19.8, 1],
                         [ 19.3, 22.9, 1],
                         [ 17.1, 21.7, 1],
                         [ 14.2, 23.1, 1],
                         [ 12.3, 25.1, 1],
                         [ 9.18, 24.0, 1],
                         [ 5.79, 24.4, 1],
                         [ 4.97, 29.9, 1],
                         [ 8.37, 28.6, 1],
                         [ 15.1, 26.3, 1],
                         [ 22.7, 26.7, 1]
                         ])
  return LandMarkMap

class Neural_Network():
  def __init__(self):
    self.LAND_MARKS = setLandMarks()
    self.rotate_cood()
    self.get_nearTree()
    #self.LAND_MARKS_ROTATE()

    #print('len : ', len(self.LAND_MARKS_ROTATE[0]))
    #print('robo_cood_x : ', self.LAND_MARKS_ROTATE[1])
    #self.write_csv()


  def rotate_cood(self):
    #ROBOT_COOD = []
    #TREE_COOD = []
    #COORDINATE = []

    q_set = []  #list for c-space samples (x,y,theta)
    x_set = np.arange(0, 35, 17.5)
    y_set = np.arange(0, 35, 35)
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
                      [C, -S, x],
                      [S, C, y],
                      [0, 0, 1]
                      ])
      # print(R_x)
      self.R_set.append(R_x)
    
    #print('R_set:{}'.format(len(R_set)))
    return 

  def get_nearTree(self):
    NearTree = []
    #nearTree = []
    for n in range(len(self.q_set)):
      num = 0
      nearTree = []
      for m in range(len(self.LAND_MARKS)):
        #print('q_set : ', self.q_set[n][0])
        #x = np.array([self.q_set[n][0], self.LAND_MARKS[m][0]])
        #y = np.array([self.q_set[n][1], self.LAND_MARKS[m][1]])
        x = np.array([self.q_set[n][0], self.q_set[n][1]])
        y = np.array([self.LAND_MARKS[m][0], self.LAND_MARKS[m][1]])
        distance = np.linalg.norm(x - y)
        if num <= 2:
          nearTree.append(np.append(self.LAND_MARKS[m], distance))
          #nearTree.append([self.LAND_MARKS[m], distance])
          #print('nearTree1 : ', nearTree)
          nearTree.sort(key=lambda x: x[3])
          #sort_nearTree = sorted(nearTree, key=lambda x: x[3])
        elif distance <= nearTree[n][3]:
          nearTree[n] = np.append(self.LAND_MARKS[m], distance)
          nearTree.sort(key=lambda x: x[3])

        #print('nearTree1 : ', sort_nearTree[n])

        num += 1
      NearTree.append(nearTree)
      #NearTree.append(sort_nearTree)
      #del nearTree

    #print('nearTreelen : ', len(NearTree))
    print('nearTree1 : ', NearTree)
    #print('nearTree1 : ', NearTree[0][0])
    #print('nearTree2 : ', NearTree[0][1])
    #print('nearTree3 : ', NearTree[0][2])
  

  def LAND_MARKS_ROTATE(self):
    self.TREE_COOD = []
    COORDINATE_ALL = []
    COORDINATE = []
    coordinate = []
    for a in range(len(self.R_set)):
      #print('R_set : ', self.R_set[a])
      for b in range(len(self.LAND_MARKS)):
        #print('LAND : ', self.LAND_MARKS[b])
        coordinate = np.dot(self.R_set[a], self.LAND_MARKS[b])
        COORDINATE.append(coordinate)
      COORDINATE_ALL.append(COORDINATE)
    self.TREE_COOD.append(COORDINATE_ALL)
    #print('coord : ', self.TREE_COOD[0][0])
    return

    #for x in np.linspace(0, 35, 2):    #351
    #  for y in np.linspace(0, 35, 2):
    #    for deg in range(2):  #360
#
    #      r = np.radians(deg)
    #      r = np.round(r,3)
    #      C = np.cos(r)
    #      S = np.sin(r)
    #      R_x = np.array([
    #                     [C, -S, x],
    #                     [S, C, y],
    #                     [0, 0, 1]
    #                     ])
    #      ROBOT_COOD.append([x, y, deg])
#
    #      for i in range(len(self.LAND_MARKS)):
    #        coordinate = np.dot(R_x, self.LAND_MARKS[i])
    #        COORDINATE.append([coordinate])
    #        #COORDINATE.append([np.dot(R_x,self.LAND_MARKS[i])])
    #      TREE_COOD.append(COORDINATE)
#
    #      #for i in self.LAND_MARKS:
    #      #  coordinate = np.dot(R_x, i)
    #      #  COORDINATE.append([np.dot(R_x, i)])
    #      #TREE_COOD.append(COORDINATE)
#
    #return ROBOT_COOD, TREE_COOD
  

  def write_csv(self):
    with open('/home/soma1/Documents/noboru/csv/dataset.csv', 'w') as file:
      writer = csv.writer(file)
      writer.writerow(['x', 'y', 'theta'])
      for a in range(len(self.q_set)):
        writer.writerow([self.q_set[a][0], self.q_set[a][1], self.q_set[a][2]])

    return

if __name__ == '__main__':
  rospy.init_node('neural_network',anonymous=True)
  node = Neural_Network()