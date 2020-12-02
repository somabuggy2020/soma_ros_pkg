#!/usr/bin/env python
import rospy
import math
import numpy as np
import itertools
import structure
import gc

from visualization_msgs.msg import MarkerArray
#from jsk_recognition_msgs.msg import BoundingBoxArray,BoundingBox

MAX_DIST = 17

def setLandMarks():
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

class coodinateSub():
  def __init__(self):
    self.LAND_MARKS = setLandMarks()
    self.TRIANGLE_LIST = self.makeTriangleList(self.LAND_MARKS)
    #rospy.Subscriber('marker', MarkerArray, self.call_back)
    rospy.Subscriber('marker_center', MarkerArray, self.call_back)
    #rospy.Subscriber('bounding_box', BoundingBoxArray, self.call_back)


  def makeTriangleList(self, _LAND_MARKS):
    MapList = []
    for i in _LAND_MARKS:
      x = round(i[0],2)
      y = round(i[1],2)
      p = structure.Point(x,y,int(i[2]))
      MapList.append(p)

    tmpTriangleList = list(itertools.combinations(MapList, 3))
    print("TriangleNUM : " + str(len(tmpTriangleList)))

    oldTriangleList = []
    for i in tmpTriangleList:
        oldTriangleList.append(structure.Triangle(i))

    TriangleList = list(filter(self.thresDist, oldTriangleList))
    print("EditTriangleNUM : " + str(len(TriangleList)))

    del MapList, tmpTriangleList, oldTriangleList
    gc.collect()

    return TriangleList


  def thresDist(self, triangle):
    a = triangle.a.dist
    if a < MAX_DIST:
      return True
    else:
      return False



  def call_back(self, center):
    if (len(center.markers) == 3):
      print("333333333333333333333333")
      print("0.x    ",center.markers[0].pose.position.x)
      print("0.y    ",center.markers[0].pose.position.y)
      print("1.x    ",center.markers[1].pose.position.x)
      print("1.y    ",center.markers[1].pose.position.y)
      print("2.x    ",center.markers[2].pose.position.x)
      print("2.y    ",center.markers[2].pose.position.y)
    else:
      return;

if __name__ == '__main__':
  rospy.init_node('coordinate_sub', anonymous=True)
  
  node = coodinateSub()
  
  rospy.spin()