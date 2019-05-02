#!/usr/bin/env python
'''
ENPM 808F 673 Spring 2019: Robot Learning
Final Project Neurla Net

Author:
Ashwin Varghese Kuruttukulam(ashwinvk94@gmail.com)
Graduate Students in Robotics,
University of Maryland, College Park
'''

from keras import layers, models, optimizers
from keras import backend as K
import numpy as np
import pickle
from matplotlib import pyplot as plt
import time
from keras.models import load_model

allData = pickle.load( open( "inputs_outputs.pkl", "rb" ) )
inputs = np.array(allData[0])
outputs = np.array(allData[1])

x_train = inputs
y_train = outputs

# print inputs[0,:]
# print len(outputs)
global action_range
global action_low
#The states contain the corners pixel values for the gate and the raw inputs from the imu
state_size = 2*4+3+3
action_range = [2,2,2]
action_low = [-1,-1,9.81]
states = layers.Input(shape=(state_size,), name='states')

# Initially only control roll and pitch
action_size = 3

# Add hidden layers
net = layers.Dense(units=32, activation='relu')(states)
net = layers.BatchNormalization()(net)
net = layers.Dense(units=64, activation='relu')(net)
net = layers.BatchNormalization()(net)
net = layers.Dense(units=32, activation='relu')(net)
raw_actions = layers.Dense(units=action_size, activation='sigmoid',name='raw_actions')(net)
actions = layers.Lambda(lambda x: (x * [0.5,2,2]) + [-0.25,-1,9.8],name='actions')(raw_actions)
# Create Keras model
model = models.Model(inputs=states, outputs=actions)
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

history = model.fit(x_train, y_train,validation_split=0.25,epochs=200, batch_size=8)

model.save('my_model.h5')

print 'predict'
print outputs[0]

start = time.time()
ip = np.array([x_train[0,:]])

model = load_model('my_model.h5')
# action_range = 14
# action_low = -2 
print ip.shape
print model.predict(ip)
end = time.time()
print(end - start)
# print(history.history.keys())
# loss_and_metrics = model.evaluate(x_test, y_test, batch_size=128)
# summarize history for accuracy
plt.plot(history.history['acc'])
plt.plot(history.history['val_acc'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()
# summarize history for loss
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()