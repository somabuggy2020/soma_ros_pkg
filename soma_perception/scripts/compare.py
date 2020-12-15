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

def main():
    # テスト用データ
    test_data = np.loadtxt('/home/soma1/Documents/noboru/csv/9_tree_detection_compare.csv', delimiter=',', comments='#')
    x_test = test_data[:, 3:9]
    #y_test_alpha = test_data[:, 12:21]
    #y_test_beta = test_data[:, 21:30]
    #y_test_gamma = test_data[:, 30:39]
    
    # モデル生成
    model_alpha = keras.models.load_model('/home/soma1/Documents/noboru/model/9_alpha.h5', compile=False)
    model_beta = keras.models.load_model('/home/soma1/Documents/noboru/model/9_beta.h5', compile=False)
    model_gamma = keras.models.load_model('/home/soma1/Documents/noboru/model/9_gamma.h5', compile=False)

    # モデルの検証・性能評価
    y_test_alpha = model_alpha.predict(x_test)
    y_test_alpha = np.round(y_test_alpha)
    y_test_beta = model_beta.predict(x_test)
    y_test_beta = np.round(y_test_beta)
    y_test_gamma = model_gamma.predict(x_test)
    y_test_gamma = np.round(y_test_gamma)

    
    x_id = test_data[:, 9:12]
    FLAT_LIST = []
    for x in range(len(x_id)):
      #print('x_id_len :', len(y_test_alpha[x]))
      #print('x_id :', y_test_alpha[x])
      for a_pos in range(len(y_test_alpha[x])):
        if y_test_alpha[x][a_pos] == 1:
          break
      for b_pos in range(len(y_test_beta[x])):
        if y_test_beta[x][b_pos] == 1:
          break
      for c_pos in range(len(y_test_gamma[x])):
        if y_test_gamma[x][c_pos] == 1:
          break
      if x_id[x][0] != a_pos+1 or x_id[x][1] != b_pos+1 or x_id[x][2] != c_pos+1:
        #flat_list = np.ravel([x_id[x][0], x_id[x][1], x_id[x][2], a_pos + 1, b_pos + 1, c_pos + 1])
        FLAT_LIST.append([x_id[x][0], x_id[x][1], x_id[x][2], a_pos + 1, b_pos + 1, c_pos + 1])
    #make_list = FLAT_LIST.reshape(1, 6)
    np.savetxt('/home/soma1/Documents/noboru/csv/9_no_match.csv', FLAT_LIST, delimiter=',', encoding='utf-8')



    #flat_list = np.ravel([y_test_alpha, y_test_beta, y_test_gamma])
    #make_list = flat_list.reshape(len(y_test_alpha), 27)
    #np.savetxt('/home/soma1/Documents/noboru/csv/9_alpha_comp.csv', make_list, delimiter=',', encoding='utf-8')



if __name__ == '__main__':
    main()