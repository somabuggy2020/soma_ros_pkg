#!/usr/bin/env python
import rospy
import math

from visualization_msgs.msg import MarkerArray

def call_back(center):
  if (len(center.markers) == 3):
    
  else:
    

if __name__ == '__main__':
  rospy.init_node('coordinate_sub', anonymous=True)
  
  #rospy.Subscriber('marker', MarkerArray, call_back)
  rospy.Subscriber('marker_center', MarkerArray, call_back)
  
  rospy.spin()