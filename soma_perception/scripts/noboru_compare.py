#!/usr/bin/env python
# -*- coding: utf-8 -*
import numpy as np
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import RMSprop
from keras.optimizers import SGD
import pandas as pd
from pandas import Series, DataFrame
from tensorflow.python.keras.models import load_model
import matplotlib.pyplot as plt
import math

import rospy
import rosparam

from std_msgs.msg import Float32MultiArray


def setLandMarks():
  #9
  LandMarkMap = np.array([
                         [2.3, 10.1, 1],
                         [4.2, 8.1, 2],
                         [5.1, 11.3, 3],
                         [7.1, 6.7, 4],
                         [9.3, 7.9, 5],
                         [10.0, 4.3, 6],
                         [11.2, 0.6, 7],
                         [12.8, 4.8, 8],
                         [12.7, 11.7, 9]
                         ])
  return LandMarkMap

class compare():
  def __init__(self):
    rospy.Subscriber('tree_list', Float32MultiArray, self.call_back)
    self.tree_id_pub = rospy.Publisher("tree_id", Float32MultiArray, queue_size=10)
    

  def call_back(self, cood):
    LAND_MARKS = setLandMarks()
    # テスト用データ
    #test_data = np.loadtxt('/home/soma1/Documents/noboru/csv/noboru_make_treelist_test.csv', delimiter=',', comments='#')
    #x_test = np.array([test_data[3:15]])
    x_test = np.array([cood.data])

    # モデル生成
    model_alpha = keras.models.load_model('/home/soma1/Documents/noboru/model/4_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_alpha.h5', compile=False)
    model_beta = keras.models.load_model('/home/soma1/Documents/noboru/model/4_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_beta.h5', compile=False)
    model_gamma = keras.models.load_model('/home/soma1/Documents/noboru/model/4_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_gamma.h5', compile=False)

    # モデルの検証・性能評価
    y_test_alpha = model_alpha.predict(x_test)
    y_test_beta = model_beta.predict(x_test)
    y_test_gamma = model_gamma.predict(x_test)

    print('a : ', y_test_alpha)
    print('b : ', y_test_beta)
    print('c : ', y_test_gamma)

    #fix id
    first_a = 0
    first_b = 0
    first_c = 0
    second_a = 0
    second_b = 0
    second_c = 0

    for n in range(9):
      if first_a < y_test_alpha[0][n]:
        first_a = y_test_alpha[0][n]
        a = n + 1
      if first_b < y_test_beta[0][n]:
        first_b = y_test_beta[0][n]
        b = n + 1
      if first_c < y_test_gamma[0][n]:
        first_c = y_test_gamma[0][n]
        c = n + 1
    print('a_id : ', a)
    print('b_id : ', b)
    print('c_id : ', c)

    #tree_id = Float32MultiArray()
    #tree_id.data = np.array([a, b, c])
    #self.tree_id_pub.publish(tree_id)

    ab = np.sqrt((LAND_MARKS[a - 1][0] - LAND_MARKS[b - 1][0])** 2 + (LAND_MARKS[a - 1][1] - LAND_MARKS[b - 1][1])** 2)
    bc = np.sqrt((LAND_MARKS[b - 1][0] - LAND_MARKS[c - 1][0])** 2 + (LAND_MARKS[b - 1][1] - LAND_MARKS[c - 1][1])** 2)
    ca = np.sqrt((LAND_MARKS[c - 1][0] - LAND_MARKS[a - 1][0])** 2 + (LAND_MARKS[c - 1][1] - LAND_MARKS[a - 1][1])** 2)    

    #if ab > 12 and bc > 12:
    if bc > 12:  
      for m in range(9):
        if second_b < y_test_beta[0][m] and first_b != y_test_beta[0][m]:
          second_b = y_test_beta[0][m]
          b = m + 1
    #if bc > 12 and ca > 12:
    if ca > 12:
      for m in range(9):
        if second_c < y_test_gamma[0][m] and first_c != y_test_gamma[0][m]:
          second_c = y_test_gamma[0][m]
          c = m + 1
    #if ca > 12 and ab > 12:
    if ab > 12:
      for m in range(9):
        if second_a < y_test_alpha[0][m] and first_a != y_test_alpha[0][m]:
          second_a = y_test_alpha[0][m] 
          a = m + 1
    print('NEW a_id : ', a)
    print('NEW b_id : ', b)
    print('NEW c_id : ', c)

    tree_id = Float32MultiArray()
    tree_id.data = np.array([a, b, c])
    self.tree_id_pub.publish(tree_id)

    return

if __name__ == '__main__':
    rospy.init_node('compare', anonymous=True)
    node = compare()
    rospy.spin()