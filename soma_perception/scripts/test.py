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
    # 訓練データ
    #x_train  入力（学習データ）
    #y_train  出力（教師データ）
    dataset = np.loadtxt('/home/soma1/Documents/noboru/csv/full_tree_detection_train.csv', delimiter=',', comments='#')
    x_train = dataset[:, 0:9]
    y_train = dataset[:, 9:12]
    
    #print(x_train.shape, y_train.shape)
    #print(y_train)

    # x_train = pd.read_csv('/home/soma1/Documents/noboru/csv/tree_detection_train.csv', usecols=[3, 4, 6, 7, 9, 10])
    # y_train = pd.read_csv('/home/soma1/Documents/noboru/csv/tree_detection_train.csv', usecols=[12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23])


    # テスト用データ
    # x_test = pd.read_csv('/home/soma1/Documents/noboru/csv/tree_detection_test.csv', usecols=[4, 5, 7, 8, 10, 11])
    x_test = np.loadtxt('/home/soma1/Documents/noboru/csv/full_tree_detection_test.csv', delimiter=',', comments='#')
    x_test = np.array([x_test[0:9]])
    
    # モデル生成
    model = Sequential()

    # 入力の次元数6, 36次元に線形変換, 活性化関数
    #model.add(Dense(units=3, activation='relu', input_dim=9))
    model.add(Dense(units=8, activation='relu', input_dim=9))

    # 3次元に線形変換, 活性化関数
    #model.add(Dense(units=(3*4), activation='softmax'))
    model.add(Dense(units=3, activation=None))
    
    # コンパイル（勾配法：adam、誤差関数：categorical_crossentropy）
    #model.compile(loss='mean_squared_error', optimizer=SGD(lr=0.1))
    model.compile(loss='mean_squared_error', optimizer='adam')

    # 構築したモデルで学習
    history = model.fit(x_train, y_train, epochs=100, verbose=1)

    # モデルの検証・性能評価
    #y_test = model.predict(x_train)
    y_test = model.predict(x_test)
    y_test = np.round(y_test)
    print(y_test)

    np.savetxt('tmp.txt',y_test,encoding='utf-8')

    model.save('/home/soma1/Documents/noboru/model/test_small.h5')

    plt.plot(history.history['loss'])
    plt.title('Model loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend(['Train'], loc='upper left')
    plt.show()

if __name__ == '__main__':
    main()