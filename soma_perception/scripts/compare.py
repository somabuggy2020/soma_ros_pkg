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

def main():
    LAND_MARKS = setLandMarks()
    # テスト用データ
    test_data = np.loadtxt('/home/soma1/Documents/noboru/csv/fil_test.csv', delimiter=',', comments='#')
    #x_test = test_data[:, 3:15]
    x_test = np.array([test_data[3:15]])
    #print('x_test : ', x_test)
    
    # モデル生成
    model_alpha = keras.models.load_model('/home/soma1/Documents/noboru/model/2_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_alpha.h5', compile=False)
    model_beta = keras.models.load_model('/home/soma1/Documents/noboru/model/2_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_beta.h5', compile=False)
    model_gamma = keras.models.load_model('/home/soma1/Documents/noboru/model/2_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_gamma.h5', compile=False)

    # モデルの検証・性能評価
    y_test_alpha = model_alpha.predict(x_test)
    #y_test_alpha = np.round(y_test_alpha)
    y_test_beta = model_beta.predict(x_test)
    #y_test_beta = np.round(y_test_beta)
    y_test_gamma = model_gamma.predict(x_test)
    #y_test_gamma = np.round(y_test_gamma)

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
      
    ab = np.sqrt((LAND_MARKS[a - 1][0] - LAND_MARKS[b - 1][0])** 2 + (LAND_MARKS[a - 1][1] - LAND_MARKS[b - 1][1])** 2)
    bc = np.sqrt((LAND_MARKS[b - 1][0] - LAND_MARKS[c - 1][0])** 2 + (LAND_MARKS[b - 1][1] - LAND_MARKS[c - 1][1])** 2)
    ca = np.sqrt((LAND_MARKS[c - 1][0] - LAND_MARKS[a - 1][0])** 2 + (LAND_MARKS[c - 1][1] - LAND_MARKS[a - 1][1])** 2)    
    
    if ab > 12 and bc > 12:
      for m in range(9):
        if second_b < y_test_beta[0][m] and first_b != y_test_beta[0][m]:
          second_b = y_test_beta[0][m]
          b = m + 1
    if bc > 12 and ca > 12:
      for m in range(9):
        if second_c < y_test_gamma[0][m] and first_c != y_test_gamma[0][m]:
          second_c = y_test_gamma[0][m]
          c = m + 1
    if ca > 12 and ab > 12:
      for m in range(9):
        if second_a < y_test_alpha[0][m] and first_a != y_test_alpha[0][m]:
          second_a = y_test_alpha[0][m] 
          a = m + 1
    print('NEW a_id : ', a)
    print('NEW b_id : ', b)
    print('NEW c_id : ', c)


    #x_id = test_data[:, 15:18]
    #x_id = np.array([test_data[15:18]])
    #FLAT_LIST = []
    #for x in range(len(x_id)):
    #  a_id = 0
    #  b_id = 0
    #  c_id = 0
    #  a_acc_max = 0
    #  b_acc_max = 0
    #  c_acc_max = 0
    #  a_acc_second = 0
    #  b_acc_second = 0
    #  c_acc_second = 0
    #  for a_pos in range(len(y_test_alpha[x])):
    #    if y_test_alpha[x][a_pos] >= a_acc_max:
    #      a_acc_second = a_acc_max
    #      a_second_id = a_id
    #      a_acc_max = y_test_alpha[x][a_pos]
    #      a_id = a_pos + 1
    #    if a_acc_max > y_test_alpha[x][a_pos] and a_acc_second < y_test_alpha[x][a_pos]:
    #      a_acc_second = y_test_alpha[x][a_pos]
    #      a_second_id = a_pos + 1
    #  for b_pos in range(len(y_test_beta[x])):
    #    if y_test_beta[x][b_pos] >= b_acc_max:
    #      b_acc_second = b_acc_max
    #      b_second_id = b_id
    #      b_acc_max = y_test_beta[x][b_pos]
    #      b_id = b_pos + 1
    #    if b_acc_max > y_test_beta[x][b_pos] and b_acc_second < y_test_beta[x][b_pos]:
    #      b_acc_second = y_test_beta[x][b_pos]
    #      b_second_id = b_pos + 1
    #  for c_pos in range(len(y_test_gamma[x])):
    #    if y_test_gamma[x][c_pos] >= c_acc_max:
    #      c_acc_second = c_acc_max
    #      c_second_id = c_id
    #      c_acc_max = y_test_gamma[x][c_pos]
    #      c_id = c_pos + 1
    #    if c_acc_max > y_test_gamma[x][c_pos] and c_acc_second < y_test_gamma[x][c_pos]:
    #      c_acc_second = y_test_gamma[x][c_pos]
    #      c_second_id = c_pos + 1
    #  if x_id[x][0] != a_id or x_id[x][1] != b_id or x_id[x][2] != c_id:
    #    FLAT_LIST.append([x_id[x][0], x_id[x][1], x_id[x][2], a_id, b_id, c_id, a_acc_max, b_acc_max, c_acc_max, a_second_id, b_second_id, c_second_id, a_acc_second, b_acc_second, c_acc_second])
    #np.savetxt('/home/soma1/Documents/noboru/csv/9_trivec_kyoku_no_match.csv', FLAT_LIST, delimiter=',')


if __name__ == '__main__':
    main()