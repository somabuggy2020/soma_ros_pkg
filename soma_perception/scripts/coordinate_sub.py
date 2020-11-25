#!/usr/bin/env python
import rospy
import math

from visualization_msgs.msg import MarkerArray
#from jsk_recognition_msgs.msg import BoundingBoxArray,BoundingBox

def call_back(center):
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
  
  #rospy.Subscriber('marker', MarkerArray, call_back)
  rospy.Subscriber('marker_center', MarkerArray, call_back)
  #rospy.Subscriber('bounding_box', BoundingBoxArray, call_back)
  
  rospy.spin()