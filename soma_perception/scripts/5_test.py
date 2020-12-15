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
    dataset = np.loadtxt('/home/soma1/Documents/noboru/csv/5_tree_detection_train.csv', delimiter=',', comments='#')
    x_train = dataset[:, 0:9]
    y_train_alpha = dataset[:, 12:17]
    y_train_beta = dataset[:, 17:22]
    y_train_gamma = dataset[:, 22:27]
    
    # テスト用データ
    test_data = np.loadtxt('/home/soma1/Documents/noboru/csv/5_tree_detection_test.csv', delimiter=',', comments='#')
    #x_test = np.array([x_test[0:9]])
    x_test = test_data[:, 0:9]
    y_test_alpha = test_data[:, 12:17]
    y_test_beta = test_data[:, 17:22]
    y_test_gamma = test_data[:, 22:27]
    
    # モデル生成
    model_alpha = Sequential()
    model_beta = Sequential()
    model_gamma = Sequential()

    # 3次元に線形変換, 活性化関数
    model_alpha.add(Dense(units=5, activation='softmax', input_dim=9))
    model_beta.add(Dense(units=5, activation='softmax', input_dim=9))
    model_gamma.add(Dense(units=5, activation='softmax', input_dim=9))

    # コンパイル（勾配法：adam、誤差関数：categorical_crossentropy）
    #model_alpha.compile(loss='categorical_crossentropy', optimizer=SGD(lr=0.1))
    model_alpha.compile(loss='categorical_crossentropy', optimizer=SGD(lr=0.1), metrics=['accuracy'])
    model_beta.compile(loss='categorical_crossentropy', optimizer=SGD(lr=0.1), metrics=['accuracy'])
    model_gamma.compile(loss='categorical_crossentropy', optimizer=SGD(lr=0.1), metrics=['accuracy'])

    # 構築したモデルで学習
    #history_alpha = model_alpha.fit(x_train, y_train_alpha, epochs=100, verbose=1)
    history_alpha = model_alpha.fit(x_train, y_train_alpha, epochs=100, verbose=1, validation_data=(x_test, y_test_alpha))
    history_beta = model_beta.fit(x_train, y_train_beta, epochs=100, verbose=1, validation_data=(x_test, y_test_beta))
    history_gamma = model_gamma.fit(x_train, y_train_gamma, epochs=100, verbose=1, validation_data=(x_test, y_test_gamma))

    # モデルの検証・性能評価
    score_alpha = model_alpha.evaluate(x_test, y_test_alpha, verbose=0)
    print('Test alpha accuracy:', score_alpha[1])
    score_beta = model_beta.evaluate(x_test, y_test_beta, verbose=0)
    print('Test beta accuracy:', score_beta[1])
    score_gamma = model_gamma.evaluate(x_test, y_test_gamma, verbose=0)
    print('Test gamma accuracy:', score_gamma[1])

    plt.plot(history_alpha.history['accuracy'])
    plt.plot(history_alpha.history['val_accuracy'])
    plt.title('model accuracy')
    plt.ylabel('accuracy')
    plt.xlabel('epoch')
    plt.legend(['train', 'test'], loc='upper left')
    plt.show()


    #y_test_alpha = model_alpha.predict(x_test)
    #y_test_alpha = np.round(y_test_alpha)
    #print(len(y_test_alpha))
    #print(y_test_alpha)
    #y_test_beta = model_beta.predict(x_test)
    #y_test_beta = np.round(y_test_beta)
    #print(y_test_beta)
    #y_test_gamma = model_gamma.predict(x_test)
    #y_test_gamma = np.round(y_test_gamma)
    #print(y_test_gamma)

    #np.savetxt('tmp.txt',y_test,encoding='utf-8')

    model_alpha.save('/home/soma1/Documents/noboru/model/a_test.h5')
    model_beta.save('/home/soma1/Documents/noboru/model/b_test.h5')
    model_gamma.save('/home/soma1/Documents/noboru/model/c_test.h5')

    #plt.plot(history_alpha.history['loss'])
    #plt.title('alpha Model loss')
    #plt.ylabel('Loss')
    #plt.xlabel('Epoch')
    #plt.legend(['Train'], loc='upper left')
    #plt.show()

if __name__ == '__main__':
    main()