#!/usr/bin/env python
'''
ENPM 808F 673 Spring 2019: Robot Learning
Final Project Neurla Net

Author:
Ashwin Varghese Kuruttukulam(ashwinvk94@gmail.com)
Graduate Students in Robotics,
University of Maryland, College Park
'''

import rosbag
from std_msgs.msg import Int32, String
import glob
import numpy as np
import pickle

def saveMsgImu(bag_name,topic_name):
	bag  = rosbag.Bag(bag_name)
	Iter = 0
	angular_velocities = []
	linear_accelerations = []
	ImuTime = []
	for topic, msg, t in bag.read_messages(topics=topic_name):
		Iter += 1
		angular_velocities.append([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
		linear_accelerations.append([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
		ImuTime.append(t)
	# print t
	# print Iter
	return angular_velocities,linear_accelerations,ImuTime

def saveMsgRateThrust(bag_name,topic_name):
	bag  = rosbag.Bag(bag_name)
	Iter = 0
	angular_rates_thrust = []
	ratesTime = []
	for topic, msg, t in bag.read_messages(topics=topic_name):
		Iter += 1
		angular_rates_thrust.append([msg.angular_rates.x,msg.angular_rates.y,msg.angular_rates.z,msg.thrust.z])
		ratesTime.append(t)
	# print t
	# print Iter
	return angular_rates_thrust,ratesTime

def saveMsgGates(bag_name,topic_name):
	bag  = rosbag.Bag(bag_name)
	Iter = 0
	markersList = []
	markersTime = []
	for topic, msg, t in bag.read_messages(topics=topic_name):
		markers = msg.markers
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
			markersList.append([marker1,marker2,marker3,marker4])
			markersTime.append(t)
		except:
			print 'marker missing'
		Iter += 1
		# angular_rates_thrust.append([t,msg.angular_rates.x,msg.angular_rates.y,msg.angular_rates.z,msg.thrust.z])
	# print t
	# print Iter
	return markersList,markersTime

def main():
	bagFiles = glob.glob('../training_data/corners_imu_rates/*.bag')
	print bagFiles
	count = 0
	allInputs = []
	allOutputs = []
	for bagFile in bagFiles:
		angular_velocities,linear_accelerations,imuTime = saveMsgImu(bagFile,'/uav/sensors/imu')
		angular_rates_thrust,ratesTime = saveMsgRateThrust(bagFile,'/uav/input/rateThrust')
		markersList,markersTime = saveMsgGates(bagFile,'/uav/camera/left/ir_beacons')
		inputs = []
		ouputs = []
		for markerTime in markersTime:
			time = markerTime
			if time in imuTime and time in ratesTime:
				markerIndex =  markersTime.index(time)
				imuIndex =  imuTime.index(time)
				ratesIndex =  ratesTime.index(time)
				markersArr = np.array(markersList[markerIndex])
				angVelArr = np.array(angular_velocities[imuIndex])
				linAccArr = np.array(linear_accelerations[imuIndex])
				inputs.append(np.concatenate((markersArr.flatten(),angVelArr,linAccArr)))
				ouputs.append(np.array([angular_rates_thrust[ratesIndex][0],angular_rates_thrust[ratesIndex][1],angular_rates_thrust[ratesIndex][3]]))
		allInputs = allInputs + inputs
		allOutputs = allOutputs + ouputs
		count += 1
		print count
	with open('inputs_outputs.pkl', 'wb') as f:
		pickle.dump([allInputs,allOutputs], f)
if __name__ == '__main__':
	main()