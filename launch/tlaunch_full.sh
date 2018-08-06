#!/bin/sh
# 2018-07-25 LLW Created for true north gyro
# First command line argument specifies active window at startup
# example "./tlaunch_full 1" will show the phins pane as acive window on launch

# launch truenorth package, truenorth.launch 
# tmux new-session -s truenorth -n TRUENORTH_ROS -d 'roslaunch truenorth truenorth.launch --screen'
# tmux set-option -t truenorth:0 remain-on-exit
echo ----------------------------------------------------------------------
echo running truenorth processes in tmux
echo usage: tconnect_full.sh      // to launch only
echo usage: tconnect_full.sh 0    // to launch and connect to window 0
echo ----------------------------------------------------------------------


# launch ROSCORE
echo 
echo launching ROSCORE in window 0
tmux new-session -s truenorth -n ROSCORE -d 'roslaunch --core --screen'
tmux set-option -t truenorth:0 remain-on-exit

echo sleeping 5 seconds...
sleep 5
echo ...done sleeping, launching all else

echo launching truenorth package, gyro_publisher node in window 1
tmux new-window -t truenorth:1 -n KVH -d 'roslaunch truenorth att_est.launch --screen'
tmux set-option -t truenorth:1 remain-on-exit

# launch phins package, phins node
echo launching phins package, phins node in window 2
tmux new-window -t truenorth:2 -n PHINS 'roslaunch phins phins.launch --screen'
tmux set-option -t truenorth:2 remain-on-exit

# launch imu_3dm_gx4 package, imu_3dm_gx4 node
echo launching imu_3dm_gx4 package, imu_3dm_gx4 node in window 3
tmux new-window -t truenorth:3 -n 3DM_GX5 'roslaunch imu_3dm_gx4 imu.launch --screen'
tmux set-option -t truenorth:3 remain-on-exit

# launch roslaunch mems_bias  package, mems_bias_publisher node
echo launching roslaunch mems_bias  package, mems_bias_publisher node in window 4
tmux new-window -t truenorth:4 -n MEMS_BIAS 'roslaunch mems_bias mems_bias.launch --screen'
tmux set-option -t truenorth:4 remain-on-exit

# launch roslaunch att_so3 package, att_so3_publisher node
echo launching roslaunch att_so3 package, att_so3_publisher node in window 5
tmux new-window -t truenorth:5 -n MEMS_ATT 'roslaunch att_so3 att.launch --screen'
tmux set-option -t truenorth:5 remain-on-exit

# launch CPU and motherboard temp and fan sensor monotoring
echo launching CPU and motherboard temp and fan sensor monotoring in window 6
tmux new-window -t truenorth:6 -n CPU_TEMP 'roslaunch truenorth truenorth_cpu_monitor.launch  --screen'
tmux set-option -t truenorth:6 remain-on-exit

# launch CPU load monitoring
# tmux new-window -t truenorth:5 -n CPU_LOAD 'rosrun libsensors_monitor libsensors_monitor'
# tmux set-option -t truenorth:5 remain-on-exit

# launch rosbag record
echo launching rosbag record in window 7
tmux new-window -t truenorth:7 -n ROSBAG 'roslaunch truenorth truenorth_rosbag.launch --screen'
tmux set-option -t truenorth:7 remain-on-exit

# list windows
echo 
echo List of tmux windows from "tmux list-windows"
tmux list-windows
echo 
# launch rosbag record hourly
# tmux new-window -t truenorth:7 -n ROSBAG_HOURLY 'roslaunch truenorth truenorth_rosbag_hourly.launch  --screen'
# tmux set-option -t truenorth:7 remain-on-exit

# remain-on-exit keeps the window open if the program running in that window crashes or ends
# This is so you can restart the process in a window with ^b-R
tmux bind-key R respawn-window
tmux bind-key A respawn-window -k
tmux bind-key X kill-server

echo "NOTE: ^b d  detaches session"
echo "NOTE: ^b R  respawns the application in the current window"
echo "NOTE: ^b A  kills and respawns the application in the current window"
echo "NOTE: ^b X  kills the server and all windows"
echo "NOTE: ^b n  next window"
echo "NOTE: ^b p  previous window"
echo 
# optional integer command line argument to this shell to select active screen
# check to see if we have an argument 
if [ "$#" -ne 0 ]
then
    echo connecting to window $1 in 5 seconds
    sleep 5
    tmux select-window -t truenorth:$1
    tmux attach-session -t truenorth
fi
