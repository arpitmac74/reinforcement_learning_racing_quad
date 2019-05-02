#!/bin/sh 
tmux new -d -s Alpha 'roscore' #0 ros
sleep 1 
tmux split-window -v -d 'rosrun joy joy_node' #1 Joystick
sleep 1

tmux select-pane -t 0
#tmux split-window -h -d 'roslaunch flightgoggles core.launch ignore_collisions:=1 render_stereo:=1' #1 simulator
tmux split-window -h -d 'roslaunch flightgoggles core.launch render_stereo:=1' #1 simulator
sleep 1

tmux select-pane -t 2
tmux split-window -h -d 'ORB_SLAM2_PATH="$PWD/../../ORB_SLAM2"; export ROS_PACKAGE_PATH=$ORB_SLAM2_PATH/Examples/ROS:${ROS_PACKAGE_PATH}; rosrun ORB_SLAM2 Stereo $ORB_SLAM2_PATH/Vocabulary/ORBvoc.txt $ORB_SLAM2_PATH/Examples/Stereo/alpha_stereo.yaml false 1' #3 ORB SLAM
sleep 1

tmux select-pane -t 3
tmux attach
