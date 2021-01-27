#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

def callback(fix, ekf, odom, rgbd_F, rgbd_B , fix_pose):
  ekf_x = ekf.pose.pose.position.x
  ekf_y = ekf.pose.pose.position.y
  odom_x = odom.pose.pose.position.x
  odom_y = odom.pose.pose.position.y
  rgbd_F_x = rgbd_F.pose.pose.position.x
  rgbd_F_y = rgbd_F.pose.pose.position.y
  rgbd_B_x = rgbd_B.pose.pose.position.x
  rgbd_B_y = rgbd_B.pose.pose.position.y
  fixpose_x = fix_pose.pose.pose.position.x
  fixpose_y = fix_pose.pose.pose.position.y

  cood_data = Float32MultiArray()
  cood_data.data = np.array([ekf_x, ekf_y, odom_x, odom_y, rgbd_F_x, rgbd_F_y, rgbd_B_x, rgbd_B_y, fixpose_x, fixpose_y])
  pub = rospy.Publisher('cood_data', Float32MultiArray, queue_size=10)
  pub.publish(cood_data)

    
if __name__=='__main__':
    rospy.init_node('pub_cood',anonymous=True)

    sub1 = message_filters.Subscriber('/fix', NavSatFix)
    sub2 = message_filters.Subscriber('/odometry/filtered', Odometry)
    sub3 = message_filters.Subscriber('/soma/odom_dr_cov', Odometry)
    sub4 = message_filters.Subscriber('/camera_F/vodom', Odometry)
    sub5 = message_filters.Subscriber('/camera_B/vodom', Odometry)
    sub6 = message_filters.Subscriber('fixed_pose', Odometry)
    fps = 1.
    delay = 1 / fps * 0.5
    ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3, sub4, sub5, sub6], 10, delay)
    ts.registerCallback(callback)

    rospy.spin()