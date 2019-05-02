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

		self.done = False
		self.joy_cb_flag = False
		self.batch_size = 32

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

		EPISODES = 2

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
		  for e in range(EPISODES):
			print 'restarting'
			self.crash_vehicle()
			print 'crashed and reset'
			# Wait for quad to reset
			self.sleep_rate.sleep()
			# state = env.reset()
			# state = np.reshape(state, [1, state_size])
			
			while self.gateInViewFlag and not self.crashFlag:
				print 'learning'
				# self.crash_vehicle()

				imu_state = [self.ang_x,self.ang_y,self.ang_z,self.lin_x,self.lin_y,self.lin_z]
				state = np.concatenate((self.markersArr.flatten(),np.array(imu_state)))
				action = self.act(state)
				reward = self.calc_reward()

				print reward
				self.rate.sleep()
			# for time in range(500):
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
		reward = self.reward_scale*(self.prev_dist_to_gate - dist_to_gate_change)
		self.prev_dist_to_gate = dist_to_gate_change

		return reward
	'''
	build network
	'''
	def _build_model(self):
		# Neural Net for Deep-Q learning Model
		model = load_model('my_model.h5')
		model._make_predict_function()
		# model = Sequential()
		# model.add(Dense(24, input_dim=self.state_size, activation='relu'))
		# model.add(Dense(24, activation='relu'))
		# model.add(Dense(self.action_size, activation='linear'))
		# model.compile(loss='mse',
		#             optimizer=Adam(lr=self.learning_rate))
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
	# def remember(self, state, action, reward, next_state, done):
	#     self.memory.append((state, action, reward, next_state, done))

	def act(self, state):
		# if np.random.rand() <= self.epsilon:
		#     return random.randrange(self.action_size)
		print state
		outputs =  self.model.predict(state.reshape(-1,14))
		roll = outputs[0][0] 
		pitch = outputs[0][1]
		yaw = 0
		thrust = outputs[0][2]
		print outputs
		ang_scale = 1
		thrust_scale = 1
		target_rate_thrust = RateThrust()
		target_rate_thrust.header.frame_id = "home"
		target_rate_thrust.header.stamp = rospy.Time.now()
		target_rate_thrust.angular_rates.x = roll
		target_rate_thrust.angular_rates.y = pitch
		target_rate_thrust.angular_rates.z = yaw
		target_rate_thrust.thrust.z = thrust
		self.rate_thrust_pub.publish(target_rate_thrust)
		# return act_values  # returns action

	# def replay(self, batch_size):
	#     minibatch = random.sample(self.memory, batch_size)
	#     for state, action, reward, next_state, done in minibatch:
	#         target = reward
	#         if not done:
	#             target = (reward + self.gamma *
	#                       np.amax(self.model.predict(next_state)[0]))
	#         target_f = self.model.predict(state)
	#         target_f[0][action] = target
	#         self.model.fit(state, target_f, epochs=1, verbose=0)
	#     if self.epsilon > self.epsilon_min:
	#         self.epsilon *= self.epsilon_decay

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
