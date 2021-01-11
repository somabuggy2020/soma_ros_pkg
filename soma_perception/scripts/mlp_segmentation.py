#!/usr/bin/env python
import os
import rospy
import roslib
import rosbag
import rosparam
import ros_numpy
import numpy as np
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2

import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras import models

INPUT_WEIGHT_DIR = '/home/soma/slope_classification/weight/200_4096/mymodel.h5'
SIZE_OF_CLOUDS = 4096
BASE_LINK = 'soma_link'

def ary_to_pc2(points, frame_id):
  cloud = PointCloud2()

  ros_dtype = sensor_msgs.PointField.FLOAT32
  dtype = np.float32
  itemsize = np.dtype(dtype).itemsize
  data = points.astype(dtype).tobytes()
  fields = [sensor_msgs.PointField(
      name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
      for i, n in enumerate('xyz')]

  cloud.header.frame_id = frame_id
  cloud.header.stamp = rospy.Time.now()
  cloud.height = 1
  cloud.width = points.shape[0]
  cloud.is_dense = False
  cloud.is_bigendian = False
  cloud.fields = fields
  cloud .point_step = (itemsize*3)
  cloud.row_step=(itemsize*3*points.shape[0])
  cloud.data = data

  return cloud


def callback(data):
  model = models.load_model(INPUT_WEIGHT_DIR)
  print(model.summary())
  ###############################
  ## convert data to numpy ary ##
  ###############################
  print('start callback')
  pc = ros_numpy.numpify(data)
  print('pc.shape', pc.shape)
  print(pc)
  test_clouds = np.zeros((pc.shape[0],6))
  test_clouds[:,0]=pc['x']
  test_clouds[:,1]=pc['y']
  test_clouds[:,2]=pc['z']
  test_clouds[:,3]=pc['normal_x']   
  test_clouds[:,4]=pc['normal_y']
  test_clouds[:,5]=pc['normal_z']
  print('test_clouds.shape', test_clouds.shape)
  print(test_clouds)

  ##################################
  ## choice specific number of pc ##
  ##################################
  N = SIZE_OF_CLOUDS
  test_clouds = test_clouds[np.random.choice(test_clouds.shape[0], N, replace=False), :]
  test_clouds = test_clouds.reshape(-1, N, 6)
  print('random choiced test_clouds.shape', test_clouds.shape)

  #############
  ## predict ##
  #############
  raw_predictions = model.predict(test_clouds)
  predictions = np.round(raw_predictions)
  print('predictions', predictions.shape)
  
  test_clouds = test_clouds.reshape(N,6)
  predictions = predictions.reshape(N,1)
  print('test', test_clouds.shape, 'predictions', predictions.shape)
  predicted_ground = np.empty((0, 3))
  predicted_slope = np.empty((0, 3))
  tmp = np.empty((0,7))
  
  test_clouds = np.hstack((test_clouds, predictions))
  print('concatenate', test_clouds.shape)
  tmp = test_clouds[test_clouds[:,6]==0]
  predicted_ground = tmp[:, 0:3]
  tmp = test_clouds[test_clouds[:,6]==1]
  predicted_slope = tmp[:, 0:3]
  print('ground', predicted_ground.shape, 'slope', predicted_slope.shape)

  ##########################
  ## publish ground,slope ##
  ##########################
  ground = ary_to_pc2(predicted_ground, frame_id=BASE_LINK)
  slope = ary_to_pc2(predicted_slope, frame_id=BASE_LINK)

  ground_pub.publish(ground)
  slope_pub.publish(slope)




if __name__ == "__main__":
  rospy.init_node('listener', anonymous=True)
  print("Initialized")

  ground_pub = rospy.Publisher('ground', PointCloud2, queue_size=1)
  slope_pub = rospy.Publisher('slope', PointCloud2, queue_size=1)

  rospy.Subscriber("cloud_normal", PointCloud2, callback)
  rospy.spin()