#!/bin/sh 
tmux new -d -s Alpha 'roscore' #0 ros
sleep 3
tmux split-window -v -d 'rosrun joy joy_node' #1 Joystick
sleep 1

# tmux select-pane -t 0
# tmux split-window -h -d 'roslaunch test3 filtered_imu_manual.launch' #1 rate publisher
# sleep 1

tmux select-pane -t 2
#tmux split-window -h -d 'roslaunch flightgoggles core.launch ignore_collisions:=1' #2 simulator
tmux split-window -h -d 'roslaunch flightgoggles core.launch' #2 simulator
sleep 1

tmux select-pane -t 3
tmux attach
