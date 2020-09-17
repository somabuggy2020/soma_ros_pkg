#!/usr/bin/env python
"""
Dead recogning node
input: /soma/ut
output: /soma/odom_dr
"""
import rospy
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import socket
import signal
from math import *

WHEEL_BASE = 1.4

#frame id
frame_id = 'odom_dr'
base_link = 'soma_link'
#subscribe topic
ut_topic = '/soma/ut'
#publish topic
odom_pub_topic = '/soma/odom_dr'

U_t = [0.0, 0.0]        #lambda v
X_t = [0.0, 0.0, 0.0]   #x,y,theta estimated
t = rospy.Time()

cont = True

def handler(signal, frame):
    global cont
    cont = False

def callback(data):
#    rospy.loginfo(data.data)
    global t

    #time
    now = rospy.get_rostime()
    dt = (now.secs + now.nsecs/1000000000) - (t.secs + t.nsecs/1000000000)

    #Dead Recogning
    if U_t[0] != 0.0 and U_t[1] != 0.0:
        omega = U_t[1]*sin(U_t[0])/WHEEL_BASE
        X_t[0] = X_t[0] - U_t[1]/omega*sin(X_t[2]) + U_t[1]/omega*sin(X_t[2] + omega*dt)
        X_t[1] = X_t[1] + U_t[1]/omega*cos(X_t[2]) - U_t[1]/omega*cos(X_t[2] + omega*dt)
        X_t[2] = X_t[2] + omega*dt
    elif U_t[0] == 0.0 and U_t[1] != 0.0:
        X_t[0] = X_t[0] + U_t[1]*dt*cos(X_t[2])
        X_t[1] = X_t[1] + U_t[1]*dt*sin(X_t[2])
        X_t[2] = X_t[2]
    else:
        pass

    t = now

    rospy.loginfo('ut:{}'.format(U_t))
    rospy.loginfo('dt:{}'.format(dt))
    rospy.loginfo('Xt:{}'.format(X_t))

    #shift
    U_t[0] = data.data[0]
    U_t[1] = data.data[1]


if __name__=='__main__':

    try:
        rospy.init_node('soma_dead_recogning_node',anonymous=True)
    except rospy.exceptions.ROSInitException:
        rospy.logfatal('not existing ros master')
        exit(-1)

    t = rospy.get_rostime()

    #subsribers
    rospy.Subscriber(ut_topic, Float32MultiArray, callback)

    #publishers
    odom_pub = rospy.Publisher(odom_pub_topic, Odometry, queue_size=3)
    trans_pub = rospy.Publisher('/soma/odom_dr2pub', TransformStamped, queue_size=3)

    odom_broadcaster = TransformBroadcaster()

    while not rospy.is_shutdown() and cont:

        q = quaternion_from_euler(0.0, 0.0, X_t[2])

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = frame_id
        odom.child_frame_id = base_link

        odom.pose.pose.position.x = X_t[0]
        odom.pose.pose.position.y = X_t[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        odom_pub.publish(odom)

        pos = (X_t[0], X_t[1], 0.0)
        quat = (q[0], q[1], q[2], q[3])
        odom_broadcaster.sendTransform(pos, quat, rospy.Time.now(), base_link, frame_id)

        rospy.sleep(0.03)
        # rospy.spinOnce()