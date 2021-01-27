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
    dataset = np.loadtxt('/home/soma1/Documents/noboru/csv/65m_trivec_kyoku_NEWrange_9_tree_detection_train.csv', delimiter=',', comments='#')
    x_train = dataset[:, 3:15]
    y_train_alpha = dataset[:, 18:27]
    y_train_beta = dataset[:, 27:36]
    y_train_gamma = dataset[:, 36:45]
    
    # テスト用データ
    test_data = np.loadtxt('/home/soma1/Documents/noboru/csv/65m_trivec_kyoku_NEWrange_9_tree_detection_test.csv', delimiter=',', comments='#')
    x_test = test_data[:, 3:15]
    y_test_alpha = test_data[:, 18:27]
    y_test_beta = test_data[:, 27:36]
    y_test_gamma = test_data[:, 36:45]
    
    # モデル生成
    model_alpha = Sequential()
    model_beta = Sequential()
    model_gamma = Sequential()

    # 3次元に線形変換, 活性化関数
    #model_alpha.add(Dense(units=9, activation='softmax', input_dim=12))
    #model_beta.add(Dense(units=9, activation='softmax', input_dim=12))
    #model_gamma.add(Dense(units=9, activation='softmax', input_dim=12))

    model_alpha.add(Dense(units=14, activation='sigmoid', input_dim=12)) #None
    model_beta.add(Dense(units=14, activation='sigmoid', input_dim=12))
    model_gamma.add(Dense(units=14, activation='sigmoid', input_dim=12))

    model_alpha.add(Dense(units=9, activation='softmax'))
    model_beta.add(Dense(units=9, activation='softmax'))
    model_gamma.add(Dense(units=9, activation='softmax'))

    # コンパイル（勾配法：adam、誤差関数：categorical_crossentropy）
    #model_alpha.compile(loss='categorical_crossentropy', optimizer=SGD(lr=0.1))
    model_alpha.compile(loss='categorical_crossentropy', optimizer=SGD(lr=0.1), metrics=['accuracy'])   #lr=0.1
    model_beta.compile(loss='categorical_crossentropy', optimizer=SGD(lr=0.1), metrics=['accuracy'])
    model_gamma.compile(loss='categorical_crossentropy', optimizer=SGD(lr=0.1), metrics=['accuracy'])

    # 構築したモデルで学習
    #history_alpha = model_alpha.fit(x_train, y_train_alpha, epochs=100, verbose=1)
    EPOCH = 50
    history_alpha = model_alpha.fit(x_train, y_train_alpha, epochs=EPOCH, verbose=1, validation_data=(x_test, y_test_alpha))
    history_beta = model_beta.fit(x_train, y_train_beta, epochs=EPOCH, verbose=1, validation_data=(x_test, y_test_beta))
    history_gamma = model_gamma.fit(x_train, y_train_gamma, epochs=EPOCH, verbose=1, validation_data=(x_test, y_test_gamma))

    # モデルの検証・性能評価
    score_alpha = model_alpha.evaluate(x_test, y_test_alpha, verbose=0)
    print('Test alpha accuracy:', score_alpha[1])
    score_beta = model_beta.evaluate(x_test, y_test_beta, verbose=0)
    print('Test beta accuracy:', score_beta[1])
    score_gamma = model_gamma.evaluate(x_test, y_test_gamma, verbose=0)
    print('Test gamma accuracy:', score_gamma[1])

    model_alpha.save('/home/soma1/Documents/noboru/model/4_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_alpha.h5')
    model_beta.save('/home/soma1/Documents/noboru/model/4_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_beta.h5')
    model_gamma.save('/home/soma1/Documents/noboru/model/4_65m_sigmoid_kyoku_NEWrange_trivec_9_epoch50_node14_gamma.h5')


    #fig, [plt_alpha, plt_beta, plt_gamma] = plt.subplots(1, 3, figsize=(15, 4))
    fig, [[plt_alpha, plt_beta, plt_gamma],[plt_alpha_loss, plt_beta_loss, plt_gamma_loss]] = plt.subplots(2, 3, figsize=(10, 6))
    
    #print(history_alpha.history)
    plt_alpha.plot(history_alpha.history['acc'])
    plt_alpha.plot(history_alpha.history['val_acc'])
    plt_alpha.set_title('alpha accuracy')
    plt_alpha.set_ylabel('accuracy')
    plt_alpha.set_xlabel('epoch')
    plt_alpha.legend(['train', 'test'], loc='lower right')

    plt_beta.plot(history_beta.history['acc'])
    plt_beta.plot(history_beta.history['val_acc'])
    plt_beta.set_title('beta accuracy')
    plt_beta.set_ylabel('accuracy')
    plt_beta.set_xlabel('epoch')
    plt_beta.legend(['train', 'test'], loc='lower right')

    plt_gamma.plot(history_gamma.history['acc'])
    plt_gamma.plot(history_gamma.history['val_acc'])
    plt_gamma.set_title('gamma accuracy')
    plt_gamma.set_ylabel('accuracy')
    plt_gamma.set_xlabel('epoch')
    plt_gamma.legend(['train', 'test'], loc='lower right')

    plt_alpha_loss.plot(history_alpha.history['loss'])
    plt_alpha_loss.plot(history_alpha.history['val_loss'])
    plt_alpha_loss.set_title('alpha loss')
    plt_alpha_loss.set_ylabel('Loss')
    plt_alpha_loss.set_xlabel('Epoch')
    plt_alpha_loss.legend(['train', 'test'], loc='upper right')

    plt_beta_loss.plot(history_beta.history['loss'])
    plt_beta_loss.plot(history_beta.history['val_loss'])
    plt_beta_loss.set_title('beta loss')
    plt_beta_loss.set_ylabel('Loss')
    plt_beta_loss.set_xlabel('Epoch')
    plt_beta_loss.legend(['train', 'test'], loc='upper right')

    plt_gamma_loss.plot(history_gamma.history['loss'])
    plt_gamma_loss.plot(history_gamma.history['val_loss'])
    plt_gamma_loss.set_title('gamma loss')
    plt_gamma_loss.set_ylabel('Loss')
    plt_gamma_loss.set_xlabel('Epoch')
    plt_gamma_loss.legend(['train', 'test'], loc='upper right')

    plt.subplots_adjust(wspace=0.4, hspace=0.6)
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

    

if __name__ == '__main__':
    main()