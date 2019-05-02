#!/usr/bin/env python2

'''
This file just publishes input from joy to quad. (Direct rate publisher).
'''

import rospy
from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Joy,Imu
import tf.transformations
import sys
from keras.models import load_model
from flightgoggles.msg import IRMarkerArray
import numpy as np


class test:
    def __init__(self):
        # Rate init
        # DECIDE ON PUBLISHING RATE
        self.rate = rospy.Rate(120.0)  # MUST be more then 2Hz
        self.joy_cb_flag = False
        self.imu_cb_flag = False
        self.attitude_thrust_pub = rospy.Publisher("/uav/input/rateThrust", RateThrust, queue_size=10)
        self.model = load_model('my_model.h5')
        self.model._make_predict_function()
        attitude_target_sub = rospy.Subscriber("/joy",Joy,self.joy_cb)
        attitude_target_sub = rospy.Subscriber("/uav/sensors/imu",Imu,self.imu_cb)
        attitude_target_sub = rospy.Subscriber("/uav/camera/left/ir_beacons",IRMarkerArray,self.gate_cb)
        self.attitude_thrust_pub = rospy.Publisher("/uav/input/rateThrust", RateThrust, queue_size=10)

        thrust_scale = 2
        ang_scale = 1
        counter = 0
        while not rospy.is_shutdown():
            # if(self.joy_cb_flag == True):
            target_attitude_thrust = RateThrust()
            target_attitude_thrust.header.frame_id = "home"
            target_attitude_thrust.header.stamp = rospy.Time.now()
            target_attitude_thrust.angular_rates.x = 0
            target_attitude_thrust.angular_rates.y = 0
            target_attitude_thrust.angular_rates.z = 0
            target_attitude_thrust.thrust.z = 10.5
            self.attitude_thrust_pub.publish(target_attitude_thrust)
            self.rate.sleep()
            counter += 1
            if counter>20:
                break

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
        # try:
        marker1,marker2,marker3,marker4
        markersArr = np.array([marker1,marker2,marker3,marker4])
        if self.imu_cb_flag:
            imu_state = [self.ang_x,self.ang_y,self.ang_z,self.lin_x,self.lin_y,self.lin_z]
            nn_input = np.concatenate((markersArr.flatten(),np.array(imu_state)))
            print nn_input
            outputs =  self.model.predict(nn_input.reshape(-1,14))
            roll = outputs[0][0] 
            pitch = outputs[0][1]
            yaw = 0
            thrust = outputs[0][2]
            print outputs
            ang_scale = 1
            thrust_scale = 1
            target_attitude_thrust = RateThrust()
            target_attitude_thrust.header.frame_id = "home"
            target_attitude_thrust.header.stamp = rospy.Time.now()
            target_attitude_thrust.angular_rates.x = roll
            target_attitude_thrust.angular_rates.y = pitch
            target_attitude_thrust.angular_rates.z = yaw
            target_attitude_thrust.thrust.z = thrust
            self.attitude_thrust_pub.publish(target_attitude_thrust)
        # markersList.append([marker1,marker2,marker3,marker4])
        # markersTime.append(t)

        # except:
        #     print 'marker missing'

def main(args):
    rospy.init_node('offb_node', anonymous=True)
    ic = test()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)