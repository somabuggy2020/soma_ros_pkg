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

def main():
    # テスト用データ
    test_data = np.loadtxt('/home/soma1/Documents/noboru/csv/trivec_kyoku_9_tree_detection_test.csv', delimiter=',', comments='#')
    x_test = test_data[:, 3:15]
    
    # モデル生成
    model_alpha = keras.models.load_model('/home/soma1/Documents/noboru/model/kyoku_trivec_9_alpha.h5', compile=False)
    model_beta = keras.models.load_model('/home/soma1/Documents/noboru/model/kyoku_trivec_9_beta.h5', compile=False)
    model_gamma = keras.models.load_model('/home/soma1/Documents/noboru/model/kypku_trivec_9_gamma.h5', compile=False)

    # モデルの検証・性能評価
    y_test_alpha = model_alpha.predict(x_test)
    #y_test_alpha = np.round(y_test_alpha)
    y_test_beta = model_beta.predict(x_test)
    #y_test_beta = np.round(y_test_beta)
    y_test_gamma = model_gamma.predict(x_test)
    #y_test_gamma = np.round(y_test_gamma)
    
    x_id = test_data[:, 15:18]
    FLAT_LIST = []
    for x in range(len(x_id)):
      a_id = 0
      b_id = 0
      c_id = 0
      a_acc_max = 0
      b_acc_max = 0
      c_acc_max = 0
      a_acc_second = 0
      b_acc_second = 0
      c_acc_second = 0
      for a_pos in range(len(y_test_alpha[x])):
        if y_test_alpha[x][a_pos] >= a_acc_max:
          a_acc_second = a_acc_max
          a_second_id = a_id
          a_acc_max = y_test_alpha[x][a_pos]
          a_id = a_pos + 1
        if a_acc_max > y_test_alpha[x][a_pos] and a_acc_second < y_test_alpha[x][a_pos]:
          a_acc_second = y_test_alpha[x][a_pos]
          a_second_id = a_pos + 1
      for b_pos in range(len(y_test_beta[x])):
        if y_test_beta[x][b_pos] >= b_acc_max:
          b_acc_second = b_acc_max
          b_second_id = b_id
          b_acc_max = y_test_beta[x][b_pos]
          b_id = b_pos + 1
        if b_acc_max > y_test_beta[x][b_pos] and b_acc_second < y_test_beta[x][b_pos]:
          b_acc_second = y_test_beta[x][b_pos]
          b_second_id = b_pos + 1
      for c_pos in range(len(y_test_gamma[x])):
        if y_test_gamma[x][c_pos] >= c_acc_max:
          c_acc_second = c_acc_max
          c_second_id = c_id
          c_acc_max = y_test_gamma[x][c_pos]
          c_id = c_pos + 1
        if c_acc_max > y_test_gamma[x][c_pos] and c_acc_second < y_test_gamma[x][c_pos]:
          c_acc_second = y_test_gamma[x][c_pos]
          c_second_id = c_pos + 1
      if x_id[x][0] != a_id or x_id[x][1] != b_id or x_id[x][2] != c_id:
        FLAT_LIST.append([x_id[x][0], x_id[x][1], x_id[x][2], a_id, b_id, c_id, a_acc_max, b_acc_max, c_acc_max, a_second_id, b_second_id, c_second_id, a_acc_second, b_acc_second, c_acc_second])
    np.savetxt('/home/soma1/Documents/noboru/csv/9_trivec_kyoku_no_match.csv', FLAT_LIST, delimiter=',')


if __name__ == '__main__':
    main()