# https://towardsdatascience.com/building-our-first-neural-network-in-keras-bdc8abbc17f5
import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
from keras.models import Sequential
from keras.layers import Dense
from keras import optimizers, callbacks
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn import preprocessing
from sklearn.model_selection import cross_val_score, KFold, train_test_split

dataset = pd.read_csv('data.csv')
dataset.head(10)

# split into input and output and train and test
# left-us, front-left-us, front-us, front-right-us, right-us, rear-left-ir, rear-ir, rear-right-ir, left-ir, right-ir, front-left-ir, front-right-ir, front-ir, x, z, theta
# 0:5, 8:13 -> less features (ignored back sensors)
# input -> 5 sharp sensors + 8 ir sensors
# output -> x, z, theta
X = dataset.iloc[:,np.r_[0:5, 8:13]].values
y = dataset.iloc[:,13:].values

# Normalize data - changes values to a common scale without distorting differences in the ranges of values
sc = preprocessing.StandardScaler()
print(X[0])
X = sc.fit_transform(X)
# print(X[0])

X_train, X_test, y_train, y_test = train_test_split(X,y, test_size=0.2)             # test is 20% of dataset
X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.2)  # from train dataset, 20% is for validation 

# base Model
model = Sequential()
model.add(Dense(250, input_dim=10, kernel_initializer="he_uniform", activation='relu'))  # input dim is the number of sensors
model.add(Dense(220, kernel_initializer="he_uniform",activation='relu'))                 # hidden layer
model.add(Dense(200, kernel_initializer="he_uniform", activation='relu'))                # hidden layer
model.add(Dense(180, kernel_initializer="he_uniform", activation='relu'))                # hidden layer
model.add(Dense(160, kernel_initializer="he_uniform", activation='relu'))                # hidden layer
model.add(Dense(140, kernel_initializer="he_uniform", activation='relu'))                # hidden layer
model.add(Dense(120, kernel_initializer="he_uniform", activation='relu'))                # hidden layer
model.add(Dense(100, kernel_initializer="he_uniform", activation='relu'))                # hidden layer
model.add(Dense(3, kernel_initializer="he_uniform"))                                    # output dim is x, z and theta

lr_values = []

class CustomCallback(callbacks.Callback):
    def on_epoch_end(self, epoch, logs=None):
        print("\nEpoch " + str(epoch) + " lr = " + str(self.model.optimizer.lr.numpy()))
        lr_values.append(self.model.optimizer.lr.numpy())

reduce_lr = callbacks.ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=50, min_lr=0.00001)

model.compile(loss="mean_squared_error", metrics=["accuracy"], optimizer='Adam')
history = model.fit(X_train, y_train, validation_data=(X_val, y_val), callbacks=[reduce_lr, CustomCallback()], epochs=1500, batch_size=300)
res = model.evaluate(X_test, y_test)
print("\nLoss = " + str(res[0]))
print("Test accuracy = " + str(res[1]))
print(model.summary())

fig, (plt1, plt2, plt3) = plt.subplots(3)
# Accuracy plot
plt1.plot(history.history['accuracy'])
plt1.plot(history.history['val_accuracy'])
plt1.set_title('Model accuracy')
plt1.set(xlabel='Epoch', ylabel='Accuracy')
plt1.set_yticks(np.arange(0, 1, 0.25))
plt1.legend(['Train', 'Test'], loc='upper left')
# Loss plot
plt2.plot(history.history['loss']) 
plt2.plot(history.history['val_loss']) 
plt2.set_title('Model loss')
plt2.set(xlabel='Epoch', ylabel='Loss')
plt2.set_yticks(np.arange(min(history.history['loss']), max(history.history['loss']), 0.25))
plt2.legend(['Train', 'Test'], loc='upper left') 
# Learning rate plot
plt3.plot(lr_values) 
plt3.set_title('Learning rate')
plt3.set(xlabel='Epoch', ylabel='Learning Rate')
plt3.legend(['Learning rate'], loc='upper left') 

plt.tight_layout()
plt.show()
