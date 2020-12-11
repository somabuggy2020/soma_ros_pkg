# -*- coding: utf-8 -*
import numpy as np
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import RMSprop
import pandas as pd
from pandas import Series,DataFrame

def main():
    # 訓練データ
    #x_train = np.array([[0,0],[0,1],[1,0],[1,1]]) # 入力（学習データ）
    #y_train = np.array([[0,0],[0,0],[0,0],[1,1]]) # 出力（教師データ）
    x_train = pd.read_csv('/home/soma1/Documents/noboru/csv/dataset_test.csv', usecols=[3, 4, 6, 7, 9, 10])
    y_train = pd.read_csv('/home/soma1/Documents/noboru/csv/dataset_test.csv', usecols=[0, 1, 2])

    # テスト用データ
    #x_test = np.array([[150,70],[175,66]])
    x_test = pd.read_csv('/home/soma1/Documents/noboru/csv/dataset_test2.csv', usecols=[3, 4, 6, 7, 9, 10])

    # モデル生成
    model = Sequential()

    # 入力の次元数2, 3次元に線形変換, 活性化関数（シグモイド）
    model.add(Dense(units=36, activation='relu', input_dim=6))

    # 2次元に線形変換, 活性化関数（softmax）
    model.add(Dense(units=3, activation=None))
   # コンパイル（勾配法：adam、誤差関数：categorical_crossentropy）
    model.compile(loss='mean_squared_error', optimizer='adam')

    # 構築したモデルで学習
    model.fit(x_train, y_train, epochs=100000)

    # モデルの検証・性能評価
    # y_test = model.predict(x_test)
    # print(y_test)


if __name__ == '__main__':
    main()