# -*- coding: utf-8 -*
import numpy as np
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import RMSprop
import pandas as pd
from pandas import Series, DataFrame
from tensorflow.python.keras.models import load_model

def main():
    # 訓練データ
    #x_train  入力（学習データ）
    #y_train  出力（教師データ）
    x_train = pd.read_csv('/home/soma1/Documents/noboru/csv/dataset.csv', usecols=[3, 4, 6, 7, 9, 10])
    y_train = pd.read_csv('/home/soma1/Documents/noboru/csv/dataset.csv', usecols=[0, 1, 2])
    
    #model = load_model('/home/soma1/Documents/noboru/model/test2.h5')

    # テスト用データ
    #x_test = pd.read_csv('/home/soma1/Documents/noboru/csv/dataset_test2.csv', usecols=[3, 4, 6, 7, 9, 10])

    # モデル生成
    model = Sequential()

    # 入力の次元数6, 36次元に線形変換, 活性化関数
    model.add(Dense(units=36, activation='relu', input_dim=6))

    # 3次元に線形変換, 活性化関数
    model.add(Dense(units=3, activation=None))
    
    # コンパイル（勾配法：adam、誤差関数：categorical_crossentropy）
    model.compile(loss='mean_squared_error', optimizer='adam')

    # 構築したモデルで学習
    model.fit(x_train, y_train, epochs=100)

    # モデルの検証・性能評価
    #y_test = model.predict(x_test)
    #print(y_test)

    model.save('/home/soma1/Documents/noboru/model/test.h5')


if __name__ == '__main__':
    main()