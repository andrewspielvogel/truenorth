#!/bin/sh
# 2018-07-28-23 LLW Created for truenorth

# optional integer command line argument to this shell to select active screen
# check to see if we have an argument 
if [ "$#" -ne 0 ]
then
    tmux select-window -t truenorth:$1
    tmux attach-session -t truenorth
else
    tmux attach-session -t truenorth
fi

