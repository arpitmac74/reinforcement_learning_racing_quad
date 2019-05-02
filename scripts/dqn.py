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
from std_msgs.msg import Int16
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
# from mavros_msgs.msg import OverrideRCIn,RCIn
import tf.transformations
from tf2_msgs.msg import TFMessage

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
		self.model = self._build_model()

		self.done = False
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

		# Rate init
		self.rate = rospy.Rate(20.0)  # MUST be more then 20Hz
		self.moveTime = 2

		#Initializing publishers
		rospy.Subscriber("/tf", TFMessage, self.gt_callback)

		# while not rospy.is_shutdown():
		# 	for e in range(EPISODES):
		#         crash_vehicle()
		#         state = env.reset()
		#         state = np.reshape(state, [1, state_size])
		#         for time in range(500):
		#             env.render()
		#             action = agent.act(state)
		#             next_state, reward, done = step(action)
		#             # next_state, reward, done, _ = env.step(action)
		#             reward = reward if not done else -10
		#             next_state = np.reshape(next_state, [1, state_size])
		#             agent.remember(state, action, reward, next_state, done)
		#             state = next_state
		#             if done:
		#                 print("episode: {}/{}, score: {}, e: {:.2}"
		#                       .format(e, EPISODES, time, agent.epsilon))
		#                 break
		#             if len(agent.memory) > batch_size:
		#                 agent.replay(batch_size)
		#             self.rate.sleep()
		#     quit()
		# 	self.rate.sleep()

	'''
	build network
	'''
	def _build_model(self):
		# Neural Net for Deep-Q learning Model
		model = Sequential()
		model.add(Dense(24, input_dim=self.state_size, activation='relu'))
		model.add(Dense(24, activation='relu'))
		model.add(Dense(self.action_size, activation='linear'))
		model.compile(loss='mse',
					  optimizer=Adam(lr=self.learning_rate))
		return model

	def gt_callback(self, msg):
		# if msg.transforms[0].child_frame_id == "uav/imu" and msg.transforms[0].header.frame_id == "world":
		pose = msg.transforms[0].transform
		# self.curRoll, self.curPitch, self.curYaw = tf.transformations.euler_from_quaternion([pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w])
		self.curX = pose.translation.x
		self.curY = pose.translation.y
		self.curZ = pose.translation.z
		print('Position feedback')
		dist_to_gate_change = self.get_dist_change()
		print dist_to_gate_change
		print(self.curX,self.curY,self.curZ)
   #          self.gt_callback_flag = True
   #      else:
			# self.gt_callback_flag = False
	
	# def step(self,action):
	# 	# Set roool pitch and thrust using the action output
	# 	reward = get_dist_change()
	# 	check_gate_cross()
	# 	next_state, reward, done = 
	
	def get_dist_change(self):
		#Check how much the dist changed from previous value
		gate_y_pos = 3.5
		dist_to_gate = gate_y_pos-self.curY
		return dist_to_gate

	# def remember(self, state, action, reward, next_state, done):
	#     self.memory.append((state, action, reward, next_state, done))

	# def act(self, state):
	#     if np.random.rand() <= self.epsilon:
	#         return random.randrange(self.action_size)
	#     act_values = self.model.predict(state)
	#     return act_values  # returns action

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
