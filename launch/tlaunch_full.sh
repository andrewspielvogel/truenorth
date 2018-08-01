#!/bin/sh
# 2018-07-25 LLW Created for true north gyro
# First command line argument specifies active window at startup
# example "./tlaunch_full 1" will show the phins pane as acive window on launch

# launch truenorth package, truenorth.launch 
tmux new-session -s truenorth -n TRUENORTH_ROS -d 'roslaunch truenorth truenorth.launch --screen'
tmux set-option -t truenorth:0 remain-on-exit

# launch truenorth package, gyro_publisher node 
# tmux new-session -s gyro -n GYRO_PUB -d 'roslaunch truenorth att_est.launch --screen'
# launch truenorth package, truenorth.launch 
# tmux set-option -t truenorth:0 remain-on-exit

# launch phins package, phins node
# tmux new-window -t truenorth:1 -n PHINS 'roslaunch phins phins.launch --screen'
# run phins package, phins node

# launch imu_3dm_gx4 package, imu_3dm_gx4 node
# tmux new-window -t truenorth:2 -n 3DM_GX5 'roslaunch imu_3dm_gx4 imu.launch --screen'
# run imu_3dm_gx4 package, imu_3dm_gx4 node

# launch roslaunch mems_bias  package, mems_bias_publisher node
# tmux new-window -t truenorth:3 -n MEMS_BIAS 'roslaunch mems_bias mems_bias.launch --screen'
# tmux set-option -t truenorth:3 remain-on-exit

# launch roslaunch att_so3 package, att_so3_publisher node
# tmux new-window -t truenorth:4 -n MEMS_ATT 'roslaunch att_so3 att.launch --screen'
# tmux set-option -t truenorth:4 remain-on-exit

# launch CPU and motherboard temp and fan sensor monotoring
# tmux new-window -t truenorth:5 -n CPU_TEMP 'roslaunch truenorth truenorth_cpu_monitor.launch  --screen'
# tmux set-option -t truenorth:5 remain-on-exit

# launch CPU load monitoring
# tmux new-window -t truenorth:5 -n CPU_LOAD 'rosrun libsensors_monitor libsensors_monitor'
# tmux set-option -t truenorth:5 remain-on-exit

# launch rosbag record
# tmux new-window -t truenorth:6 -n ROSBAG 'roslaunch truenorth truenorth_rosbag.launch  --screen'
# tmux set-option -t truenorth:6 remain-on-exit

# launch rosbag record hourly
# tmux new-window -t truenorth:7 -n ROSBAG_HOURLY 'roslaunch truenorth truenorth_rosbag_hourly.launch  --screen'
# tmux set-option -t truenorth:7 remain-on-exit

# remain-on-exit keeps the window open if the program running in that window crashes or ends
# This is so you can restart the process in a window with ^b-R
tmux bind-key R respawn-window
tmux bind-key A respawn-window -k
tmux bind-key X kill-server


# optional integer command line argument to this shell to select active screen
if test "$2" = "c"; then
    tmux select-window -t truenorth:$1
    tmux -2 attach-session -t gyro
fi
