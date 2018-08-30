#!/bin/bash
#
# 2018-08-28 LLW Created to run post processing in an interactive  shell
source ~/.bashrc
rosrun truenorth post_process $1
printenv | grep ROS
