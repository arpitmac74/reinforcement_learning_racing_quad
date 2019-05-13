#!/usr/bin/env python2
'''
Airgilty

Updates RC commands to avoid collision with objects as detected by the obstacle_detector nodes 
If an obstacle is detected in the pitch axis, the quadcopter will move in the opposite direction
for a preset amount of time or until the obstacle is no longer in front. Whichever comes first.

Author:
Ashwin Varghese Kuruttukulam(ashwinvk94@gmail.com)
'''

import sys,tf
import rospy
from mav_msgs.msg import RateThrust
from std_msgs.msg import Int16,Empty
from sensor_msgs.msg import Joy,Imu
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import load_model
import tf.transformations
from tf2_msgs.msg import TFMessage
from flightgoggles.msg import IRMarkerArray
import numpy as np
import random
from keras import layers, models, optimizers
from keras import backend as K
import time
class rc_pub:
	def __init__(self):

		self.state_size = 14
		self.action_size = 3
		self.memory = deque(maxlen=2000)
		self.gamma = 0.95    # discount rate
		self.epsilon = 1.0  # exploration rate
		self.epsilon_min = 0.01
		self.epsilon_decay = 0.995
		self.learning_rate = 0.001
		self.gate_number = 10
		self.model = self._build_model()

		self.roll = 0
		self.pitch = 0
		self.thrust = 0

		self.roll_scale = 0.01
		self.pitch_scale = 0.1
		self.thrust_scale = 0.3

		self.done = False
		self.joy_cb_flag = False
		self.imu_cb_flag = False
		self.batch_size = 8

		#CHANGE!!!
		self.initGateDist = 4

		#Initialize flags
		self.obstacleAvoidFlag = False
		self.frontObstacleFlag = False
		self.rearObstacleFlag = False
		self.leftObstacleFlag = False
		self.rightObstacleFlag = False
		self.movePitchResetFlag = False
		self.loc_flag = False

		self.prev_dist_to_gate = 26.5
		self.reward_scale = 1

		EPISODES = 200

		# Rate init
		self.rate = rospy.Rate(20.0)  # MUST be more then 20Hz
		self.sleep_rate = rospy.Rate(2.0)  # MUST be more then 20Hz
		self.moveTime = 2

		#Initializing publishers
		attitude_target_sub = rospy.Subscriber("/joy",Joy,self.joy_cb)
		rospy.Subscriber("/tf", TFMessage, self.gt_callback)
		rospy.Subscriber("/uav/collision", Empty, self.collision_callback)
		attitude_target_sub = rospy.Subscriber("/uav/sensors/imu",Imu,self.imu_cb)
		attitude_target_sub = rospy.Subscriber("/uav/camera/left/ir_beacons",IRMarkerArray,self.gate_cb)
		self.rate_thrust_pub = rospy.Publisher("/uav/input/rateThrust", RateThrust, queue_size=10)
		self.thrust_scale = 1
		self.ang_scale = 1

		#Test predict
		# while not self.imu_cb_flag:
		# 	target_rate_thrust = RateThrust()
		# 	target_rate_thrust.header.frame_id = "home"
		# 	target_rate_thrust.header.stamp = rospy.Time.now()
		# 	target_rate_thrust.angular_rates.x = 1
		# 	target_rate_thrust.angular_rates.y = 0
		# 	target_rate_thrust.angular_rates.z = 0
		# 	target_rate_thrust.thrust.z = 10.5
		# 	self.rate_thrust_pub.publish(target_rate_thrust)
		# 	self.sleep_rate.sleep()
		# 	print 'waiting for imu data'
		# imu_state = [self.ang_x,self.ang_y,self.ang_z,self.lin_x,self.lin_y,self.lin_z]
		# next_state = np.concatenate((self.markersArr.flatten(),np.array(imu_state)))
		# self.model.predict(next_state.reshape(-1,14))
		

		# while not rospy.is_shutdown():
		# 	if(self.joy_cb_flag == True):
		# 		target_rate_thrust = RateThrust()
		# 		target_rate_thrust.header.frame_id = "home"
		# 		target_rate_thrust.header.stamp = rospy.Time.now()
		# 		target_rate_thrust.angular_rates.x = -self.ang_scale*(self.roll)
		# 		target_rate_thrust.angular_rates.y = self.ang_scale*(self.pitch)
		# 		target_rate_thrust.angular_rates.z = self.ang_scale*(self.yaw)
		# 		target_rate_thrust.thrust.z = 9.81+self.thrust_scale*self.thrust
		# 		self.rate_thrust_pub.publish(target_rate_thrust)
		# 	self.rate.sleep()
		while not rospy.is_shutdown():
			print 'restarting'
			self.crash_vehicle()
			print 'crashed and reset'
			for e in range(EPISODES):
				# Wait for quad to reset
				self.sleep_rate.sleep()
				# state = env.reset()
				# state = np.reshape(state, [1, state_size])
				counter = 0
				while 1:
					print 'learning'
					# self.crash_vehicle()

					imu_state = [self.ang_x,self.ang_y,self.ang_z,self.lin_x,self.lin_y,self.lin_z]
					state = np.concatenate((self.markersArr.flatten(),np.array(imu_state)))
					outputs =  self.model.predict(state.reshape(-1,14))
					print outputs
					action = self.act(state,counter)
					# print action
					# print type(action[0])
					self.do_action(action,counter)
					# Sleeping to find the next state
					self.rate.sleep()
					reward = self.calc_reward()
					imu_state = [self.ang_x,self.ang_y,self.ang_z,self.lin_x,self.lin_y,self.lin_z]
					next_state = np.concatenate((self.markersArr.flatten(),np.array(imu_state)))

					if self.crashFlag or not self.gateInViewFlag:
						#Give negative reward
						print 'breaking!!!'
						reward = -10
						break
					# print reward
					self.remember(state, action, reward, next_state)
					if len(self.memory) > self.batch_size:
						self.replay(self.batch_size)
				#     env.render()
				#     action = agent.act(state)
				#     next_state, reward, done = step(action)
				#     # next_state, reward, done, _ = env.step(action)
				#     reward = reward if not done else -10
				#     next_state = np.reshape(next_state, [1, state_size])
				#     agent.remember(state, action, reward, next_state, done)
				#     state = next_state
				#     if done:
				#         print("episode: {}/{}, score: {}, e: {:.2}"
				#               .format(e, EPISODES, time, agent.epsilon))
				#         break
				#     if len(agent.memory) > batch_size:
				#         agent.replay(batch_size)
				#     self.rate.sleep()
					if counter<30:
						counter += 1
				self.crash_vehicle()
				self.crashFlag = False
			self.rate.sleep()

	def joy_cb(self, state):
		#print(state)
		axes = state.axes
		#RRL
		self.thrust = axes[1]
		self.yaw = axes[0]
		self.pitch = axes[4]
		self.roll = axes[3]
		#ARC
		# self.thrust = axes[1]
		# self.yaw = axes[0]
		# self.pitch = axes[3]
		# self.roll = axes[2]
		self.joy_cb_flag = True
		

	def calc_reward(self):
		dist_to_gate_change = self.get_dist_to_gate()
		gate_dist_reward = self.reward_scale*(self.prev_dist_to_gate - dist_to_gate_change)
		reward = gate_dist_reward
		self.prev_dist_to_gate = dist_to_gate_change

		return reward
	'''
	build network
	'''
	def _build_model(self):
		# # Neural Net for Deep-Q learning Model
		# # model._make_predict_function()
		# # model.add(Dense(24, input_dim=self.state_size, activation='relu'))
		# # model.add(Dense(24, activation='relu'))
		# # model.add(Dense(self.action_size, activation='linear'))
		# # model.compile(loss='mse',
		# #             optimizer=Adam(lr=self.learning_rate))
		# model = Sequential()
		# states = layers.Input(shape=(self.state_size,), name='states')
		# net = layers.Dense(units=24, activation='relu')(states)
		# # net = layers.BatchNormalization()(net)
		# # net = layers.Dense(units=64, activation='relu')(net)
		# # # net = layers.BatchNormalization()(net)
		# net = layers.Dense(units=24, activation='relu')(net)
		# raw_actions = layers.Dense(units=self.action_size, activation='sigmoid',name='raw_actions')(net)
		# model.compile(loss='mse',
		# 			optimizer=Adam(lr=self.learning_rate))
		states = layers.Input(shape=(self.state_size,), name='states')

		# Add hidden layers
		net = layers.Dense(units=32, activation='relu')(states)
		net = layers.BatchNormalization()(net)
		net = layers.Dense(units=64, activation='relu')(net)
		net = layers.BatchNormalization()(net)
		net = layers.Dense(units=32, activation='relu')(net)
		raw_actions = layers.Dense(units=self.action_size, activation='sigmoid',name='raw_actions')(net)
		# actions = layers.Lambda(lambda x: (x * [0.5,2,2]) + [-0.25,-1,9.8],name='actions')(raw_actions)
		# Create Keras model
		model = models.Model(inputs=states, outputs=raw_actions)
		model.compile(loss='mean_squared_error', optimizer=Adam(lr=self.learning_rate))

		return model


	def collision_callback(self, msg):
		self.crashFlag = True
		# print 'collision detected'

	def gt_callback(self, msg):
		# if msg.transforms[0].child_frame_id == "uav/imu" and msg.transforms[0].header.frame_id == "world":
		pose = msg.transforms[0].transform
		# self.curRoll, self.curPitch, self.curYaw = tf.transformations.euler_from_quaternion([pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w])
		self.curX = pose.translation.x
		self.curY = pose.translation.y
		self.curZ = pose.translation.z

		self.loc_flag = True
		# print('Position feedback')
		
		# print dist_to_gate_change
		# print(self.curX,self.curY,self.curZ)
   #          self.gt_callback_flag = True
   #      else:
			# self.gt_callback_flag = False
	
	# def step(self,action):
	#   # Set roool pitch and thrust using the action output
	#   reward = get_dist_change()
	#   check_gate_cross()
	#   next_state, reward, done = 
	
	def get_dist_to_gate(self):
		#Check how much the dist changed from previous value
		gate_y_pos = 3.5
		dist_to_gate = gate_y_pos-self.curY
		return dist_to_gate

	def crash_vehicle(self):
		self.crashFlag = False
		while 1:
			if self.loc_flag:
				print self.get_dist_to_gate()
			target_rate_thrust = RateThrust()
			target_rate_thrust.header.frame_id = "home"
			target_rate_thrust.header.stamp = rospy.Time.now()
			target_rate_thrust.angular_rates.x = 1
			target_rate_thrust.angular_rates.y = 0
			target_rate_thrust.angular_rates.z = 0
			target_rate_thrust.thrust.z = 10.5
			self.rate_thrust_pub.publish(target_rate_thrust)
			if self.crashFlag:
				break
			self.rate.sleep()
		self.crashFlag = False
		print 'crashed'

	def imu_cb(self, state):
		#print(state)
		self.ang_x = state.angular_velocity.x
		self.ang_y = state.angular_velocity.y
		self.ang_z = state.angular_velocity.z
		self.lin_x = state.linear_acceleration.x
		self.lin_y = state.linear_acceleration.y
		self.lin_z = state.linear_acceleration.z
		self.imu_cb_flag = True

	def gate_cb(self,state):
		markers = state.markers
		for marker in markers:
			if marker.landmarkID.data=="Gate10":
				if marker.markerID.data=="1":
					marker1 = [marker.x,marker.y]
				if marker.markerID.data=="2":
					marker2 = [marker.x,marker.y]
				if marker.markerID.data=="3":
					marker3 = [marker.x,marker.y]
				if marker.markerID.data=="4":
					marker4 = [marker.x,marker.y]       
		try:
			marker1,marker2,marker3,marker4
			self.markersArr = np.array([marker1,marker2,marker3,marker4])
			self.gateInViewFlag = True
		except:
			print 'gate not in view'
			self.gateInViewFlag = False
	def remember(self, state, action, reward, next_state):
		self.memory.append((state, action, reward, next_state))

	def rand_outputs(self,counter):
		roll = int(round(random.uniform(0, 1)))
		pitch = int(round(random.uniform(0, 1)))
		if counter<20:
			thrust = 1
		else:
			thrust = int(round(random.uniform(0,1)))

		return [roll,pitch,thrust]
	def act(self, state,counter):
		if np.random.rand() <= self.epsilon:
			# The agent acts randomly
			return self.rand_outputs(counter)
		# if np.random.rand() <= self.epsilon:
		#     return random.randrange(self.action_size)
		outputs =  self.model.predict(state.reshape(-1,14))
		outputs = outputs[0]
		rounded_ouputs = []
		for output in outputs:
			rounded_ouputs.append(int(round(output)))
		return np.array(rounded_ouputs)  # returns action

	def do_action(self,action,counter):
		self.roll += self.roll_scale*round(action[0]) 
		self.pitch += self.pitch_scale*round(action[1]) 
		yaw = 0
		self.thrust += self.thrust_scale*round(action[2]) 
		# print [self.roll,self.pitch,self.thrust]
		ang_scale = 1
		thrust_scale = 1
		target_rate_thrust = RateThrust()
		target_rate_thrust.header.frame_id = "home"
		target_rate_thrust.header.stamp = rospy.Time.now()
		target_rate_thrust.angular_rates.x = self.roll
		target_rate_thrust.angular_rates.y = self.pitch
		target_rate_thrust.angular_rates.z = yaw
		if counter<20:
			target_rate_thrust.thrust.z = 11
		else:
			target_rate_thrust.thrust.z = self.thrust
		self.rate_thrust_pub.publish(target_rate_thrust)

	def replay(self, batch_size):
		minibatch = random.sample(self.memory, batch_size)
		for state, action, reward, next_state in minibatch:
			target = reward
			target = reward + self.gamma * np.amax(self.model.predict(next_state.reshape(-1,14))[0])
			target_f = self.model.predict(state.reshape(-1,14))
			target_f[0][action] = target
			# print target_f
			self.model.fit(state.reshape(-1,14), target_f, epochs=1, verbose=0)
		if self.epsilon > self.epsilon_min:
			self.epsilon *= self.epsilon_decay
	# def replay(self, batch_size):
	# 	minibatch = random.sample(self.memory, batch_size)
	# 	for state, action, reward, next_state in minibatch:
	# 		# target = reward
	# 		# target = (reward + self.gamma *np.amax(self.model.predict(next_state.reshape(-1,14))[0]))
	# 		# target_f = self.model.predict(state.reshape(-1,14))
	# 		# target_f[0][action] = target
	# 		nEpochs = int(5*reward)
	# 		print 'reward'
	# 		print reward
	# 		print 'training epochs'
	# 		print nEpochs
	# 		if nEpochs>0:
	# 			print state.reshape(-1,14)
	# 			print action
	# 			self.model.fit(state.reshape(-1,14), action, epochs=nEpochs, verbose=0)
	# 	if self.epsilon > self.epsilon_min:
	# 		self.epsilon *= self.epsilon_decay

	# def load(self, name):
	#     self.model.load_weights(name)

	# def save(self, name):
	#     self.model.save_weights(name)


def main(args):
	rospy.init_node('rc_pub', anonymous=True)
	ic = rc_pub()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
