# -*- coding: utf-8 -*
import numpy as np
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import RMSprop

def main():
    # 訓練データ
    x_train = np.array([[0,0],[0,1],[1,0],[1,1]]) # 入力（学習データ）
    y_train = np.array([[0,0],[0,0],[0,0],[1,1]]) # 出力（教師データ）

    # テスト用データ
    x_test = np.array([[150,70],[175,66]])

    # モデル生成
    model = Sequential()

    # 入力の次元数2, 3次元に線形変換, 活性化関数（シグモイド）
    model.add(Dense(units=3, activation='sigmoid', input_dim=2))

    # 2次元に線形変換, 活性化関数（softmax）
    model.add(Dense(units=2, activation='softmax'))
   # コンパイル（勾配法：adam、誤差関数：categorical_crossentropy）
    model.compile(loss='categorical_crossentropy', optimizer='adam')

    # 構築したモデルで学習
    model.fit(x_train, y_train, epochs=100)

    # モデルの検証・性能評価
    y_test = model.predict(x_test)
    print(y_test)


if __name__ == '__main__':
    main()