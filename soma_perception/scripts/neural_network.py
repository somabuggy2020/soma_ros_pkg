#!/usr/bin/env python

import rospy
import numpy as np
import csv

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
    self.LAND_MARKS_ROTATE = self.rotate_cood()

    #print('len : ', len(self.LAND_MARKS_ROTATE[0][1]))
    #print('robo_cood_x : ', self.LAND_MARKS_ROTATE[0][0])
    #print('robo_cood_y : ', self.LAND_MARKS_ROTATE[1][0][0])
    #print('robo_cood_z : ', self.LAND_MARKS_ROTATE[2])
    #print('robo_cood_deg : ', self.LAND_MARKS_ROTATE[0][1][2])
    self.write_csv()
    print('finish')

  def rotate_cood(self):
    ROBOT_COOD = []
    TREE_COOD = []
    LAND_MARKS_rotate = []
    #LAND_MARKS_rotate = [ROBOT_COOD, TREE_COOD]

    for x in np.linspace(0, 35, 351):    #351
      for y in np.linspace(0, 35, 351):
        for deg in range(360):  #360

          r = np.radians(deg)
          C = np.cos(r)
          S = np.sin(r)
          R_x = np.array([
                         [C, -S, x],
                         [S, C, y],
                         [0, 0, 1]
                         ])
          robot_cood = np.array([x, y, r])
          ROBOT_COOD.append(robot_cood)
          #ROBOT_COOD.append([x, y, r])
          for i in range(len(self.LAND_MARKS)):
            coordinate = np.dot(R_x, self.LAND_MARKS[i])
            TREE_COOD.append(coordinate)

          LAND_MARKS_rotate.append([ROBOT_COOD, TREE_COOD])

    return LAND_MARKS_rotate

  
  def write_csv(self):
    with open('/home/soma1/Documents/noboru/csv/dataset.csv', 'w') as file:
      writer = csv.writer(file)
      writer.writerow(['x', 'y', 'theta'])
      for a in range(len(self.LAND_MARKS_ROTATE[0][0])):
        writer.writerow([self.LAND_MARKS_ROTATE[0][0][a][0], self.LAND_MARKS_ROTATE[0][0][a][1], self.LAND_MARKS_ROTATE[0][0][a][2]])

    return

if __name__ == '__main__':
  rospy.init_node('neural_network',anonymous=True)
  node = Neural_Network()