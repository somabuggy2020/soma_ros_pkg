#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def callback_F(data_F):
 cov_F = PoseWithCovarianceStamped()
 cov_F.header = data_F.header
 #cov_L.pose.pose = data_L.pose
 cov_F.pose.pose.position = data_F.pose.position
 cov_F.pose.pose.orientation.x = data_F.pose.orientation.x + 0.96592582628
 cov_F.pose.pose.orientation.y = data_F.pose.orientation.y
 cov_F.pose.pose.orientation.z = data_F.pose.orientation.z + 0.2588190451
 cov_F.pose.pose.orientation.w = data_F.pose.orientation.w

 set_cov_L = [0.5,0,0,0,0,0,
              0,0.5,0,0,0,0,
              0,0,0.5,0,0,0,
              0,0,0,0.5,0,0,
              0,0,0,0,0.5,0,
              0,0,0,0,0,0.5]

 cov_F.pose.covariance = set_cov_F

 rospy.loginfo(cov_F)

 publisher = rospy.Publisher('pose_F',PoseWithCovarianceStamped,queue_size=10)
 publisher.publish(cov_F)

def callback_B(data_B):
 cov_B = PoseWithCovarianceStamped()
 cov_B.header = data_B.header
 #cov_R.pose.pose = data_R.pose

 cov_B.pose.pose.position.x = data_B.pose.position.x * -1
 cov_B.pose.pose.position.y = data_B.pose.position.y * -1
 cov_B.pose.pose.position.z = data_B.pose.position.z
 #cov_R.pose.pose.orientation = data_R.pose.orientation
 cov_B.pose.pose.orientation.x = data_B.pose.orientation.x + 0.96592582628
 cov_B.pose.pose.orientation.y = data_B.pose.orientation.y
 cov_B.pose.pose.orientation.z = data_B.pose.orientation.z - 0.2588190451
 cov_B.pose.pose.orientation.w = data_B.pose.orientation.w

 set_cov_B = [0.5,0,0,0,0,0,
              0,0.5,0,0,0,0,
              0,0,0.5,0,0,0,
              0,0,0,0.5,0,0,
              0,0,0,0,0.5,0,
              0,0,0,0,0,0.5]

 cov_B.pose.covariance = set_cov_B

 rospy.loginfo(cov_B)

 publisher = rospy.Publisher('pose_B',PoseWithCovarianceStamped,queue_size=10)
 publisher.publish(cov_B)

def callback_dr(odom_dr):
    #odom_dr_cov = odom_dr
    #odom_dr.pose.covariance[0] = 0.2
    #odom_dr.pose.covariance[7] = 0.3
    #odom_dr.pose.covariance[35] = 0.25

    odom_dr.pose.covariance =  [0.1,0,0,0,0,0,
                                0,0.2,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0.1]
   

    publisher = rospy.Publisher('/soma/odom_dr_cov',Odometry,queue_size=10)
    publisher.publish(odom_dr)
    
if __name__=='__main__':
    rospy.init_node('poseconverter',anonymous=True)

    rospy.Subscriber('/orbslam_f/orbslam/pose',PoseStamped,callback_F)
    rospy.Subscriber('/orbslam_b/orbslam/pose',PoseStamped,callback_B)
    rospy.Subscriber('/soma/odom_dr', Odometry, callback_dr)

    rospy.spin()
