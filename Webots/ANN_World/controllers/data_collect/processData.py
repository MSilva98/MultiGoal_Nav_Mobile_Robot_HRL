import numpy as np 
from keras.models import Sequential
from keras.layers import Dense
from sklearn import preprocessing

dataset = np.loadtxt('data.csv', delimiter = ',')

# sc = preprocessing.StandardScaler()

dataset = preprocessing.normalize(dataset)

# split into input and output and train and test
# input -> 6 sharp sensors + 8 ir sensors
x_train = dataset[0:4900,0:14]
# output -> x, y, z, theta
y_train = dataset[0:4900,14:]

x_test = dataset[4900:,0:14]
y_test = dataset[4900:,14:]

model = Sequential()
model.add(Dense(12, input_dim=14, activation='relu'))
model.add(Dense(8, activation='relu'))
model.add(Dense(4))
model.compile(loss="mae", metrics=["accuracy"], optimizer="Adam")
model.fit(x_train, y_train, epochs=300, batch_size=64)

preds = model.evaluate(x_test, y_test)

print("\nLoss = " + str(preds[0]))
print("Test accuracy = " + str(preds[1]))
