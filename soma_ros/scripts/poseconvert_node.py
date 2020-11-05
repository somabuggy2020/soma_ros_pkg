#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def callback_L(data_L):
 cov_L = PoseWithCovarianceStamped()
 cov_L.header = data_L.header
 #cov_L.pose.pose = data_L.pose
 cov_L.pose.pose.position = data_L.pose.position
 cov_L.pose.pose.orientation.x = data_L.pose.orientation.x + 0.96592582628
 cov_L.pose.pose.orientation.y = data_L.pose.orientation.y
 cov_L.pose.pose.orientation.z = data_L.pose.orientation.z + 0.2588190451
 cov_L.pose.pose.orientation.w = data_L.pose.orientation.w

 set_cov_L = [0.5,0,0,0,0,0,
              0,0.5,0,0,0,0,
              0,0,0.5,0,0,0,
              0,0,0,0.5,0,0,
              0,0,0,0,0.5,0,
              0,0,0,0,0,0.5]

 cov_L.pose.covariance = set_cov_L

 rospy.loginfo(cov_L)

 publisher = rospy.Publisher('pose_L',PoseWithCovarianceStamped,queue_size=10)
 publisher.publish(cov_L)

def callback_R(data_R):
 cov_R = PoseWithCovarianceStamped()
 cov_R.header = data_R.header
 #cov_R.pose.pose = data_R.pose

 cov_R.pose.pose.position.x = data_R.pose.position.x * -1
 cov_R.pose.pose.position.y = data_R.pose.position.y * -1
 cov_R.pose.pose.position.z = data_R.pose.position.z
 #cov_R.pose.pose.orientation = data_R.pose.orientation
 cov_R.pose.pose.orientation.x = data_R.pose.orientation.x + 0.96592582628
 cov_R.pose.pose.orientation.y = data_R.pose.orientation.y
 cov_R.pose.pose.orientation.z = data_R.pose.orientation.z - 0.2588190451
 cov_R.pose.pose.orientation.w = data_R.pose.orientation.w

 set_cov_R = [0.5,0,0,0,0,0,
              0,0.5,0,0,0,0,
              0,0,0.5,0,0,0,
              0,0,0,0.5,0,0,
              0,0,0,0,0.5,0,
              0,0,0,0,0,0.5]

 cov_R.pose.covariance = set_cov_R

 rospy.loginfo(cov_R)

 publisher = rospy.Publisher('pose_R',PoseWithCovarianceStamped,queue_size=10)
 publisher.publish(cov_R)

def callback_dr(odom_dr):
    #odom_dr_cov = odom_dr
    #odom_dr.pose.covariance[0] = 0.2
    #odom_dr.pose.covariance[7] = 0.3
    #odom_dr.pose.covariance[35] = 0.25

    odom_dr.pose.covariance =  [0.2,0,0,0,0,0,
                                0,0.3,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0.25]
   

    publisher = rospy.Publisher('/soma/odom_dr_cov',Odometry,queue_size=10)
    publisher.publish(odom_dr)
    
if __name__=='__main__':
    rospy.init_node('poseconverter',anonymous=True)

    rospy.Subscriber('/orbslam_l/orbslam/pose',PoseStamped,callback_L)
    rospy.Subscriber('/orbslam_r/orbslam/pose',PoseStamped,callback_R)
    rospy.Subscriber('/soma/odom_dr', Odometry, callback_dr)

    rospy.spin()
