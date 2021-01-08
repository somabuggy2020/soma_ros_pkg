#!/usr/bin/env python
import os
import rospy
import roslib
import rosbag
import rosparam
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2

import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras import models

INPUT_WEIGHT_DIR = '/home/soma/slope_classification/weight/4096/mymodel.h5'
SIZE_OF_CLOUDS = 4096

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
  predicted_ground = np.empty((0, 6))
  predicted_slope = np.empty((0, 6))
  raw_predictions = model.predict(test_clouds)
  predictions = np.round(raw_predictions)
  print('predictions', predictions.shape)
  
  for j in range(test_clouds.shape[1]):
    if predictions[0,j] == 0:
      predicted_ground = np.vstack((predicted_ground, test_clouds[0,j]))
    else:
      predicted_slope = np.vstack((predicted_slope, test_clouds[0,j]))
    j = j + 1
  print('ground', predicted_ground.shape, 'slope', predicted_slope.shape)

  ################################
  ## convert ary to sensor_msgs ##
  ################################
  ground = PointCloud2()
  slope = PointCloud2()
  ground.data = str(predicted_ground)
  slope.data = str(predicted_slope)

  ground_pub.publish(ground)
  slope_pub.publish(slope)





if __name__ == "__main__":
  rospy.init_node('listener', anonymous=True)
  print("Initialized")

  # model = models.load_model(INPUT_WEIGHT_DIR)
  # print(model.summary())

  ground_pub = rospy.Publisher('ground', PointCloud2, queue_size=3)
  slope_pub = rospy.Publisher('slope', PointCloud2, queue_size=3)

  rospy.Subscriber("normal_cloud", PointCloud2, callback)
  rospy.spin()