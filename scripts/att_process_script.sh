#!/bin/bash

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p ~/log/$3/configs
    mkdir -p ~/log/$3/processed
    mkdir -p ~/log/$3/pdfs
    
    KVH=/home/spiels/log/$3/$4.KVH
    CSV=~/log/$3/processed/$1.csv
    CONFIG=~/log/$3/configs/$1.m
    PDF=~/log/$3/pdfs/$1.pdf
    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -k $2 -z $5

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $1

}


DIR=RSS2018
LOG=2017_12_13_15_16
HZ=5000

K=[.05,0,100,0,0,0]
EXP=exp1
process_att $EXP $K $DIR $LOG $HZ

K=[.05,0,100,0,0,100]
EXP=exp2
process_att $EXP $K $DIR $LOG $HZ

K=[.05,0,10,0,0,50]
EXP=exp3
process_att $EXP $K $DIR $LOG $HZ

K=[.005,0,100,0,0,0]
EXP=exp12
process_att $EXP $K $DIR $LOG $HZ

K=[.05,0,1000,0,0,500]
EXP=exp4
process_att $EXP $K $DIR $LOG $HZ

K=[.05,0,1000,0,0,1000]
EXP=exp5
process_att $EXP $K $DIR $LOG $HZ

K=[.05,0,1000,0,0,5000]
EXP=exp6
process_att $EXP $K $DIR $LOG $HZ

K=[.005,0,1000,0,0,5000]
EXP=exp7
process_att $EXP $K $DIR $LOG $HZ

K=[.001,0,1000,0,0,5000]
EXP=exp8
process_att $EXP $K $DIR $LOG $HZ

K=[.0005,0,1000,0,0,5000]
EXP=exp9
process_att $EXP $K $DIR $LOG $HZ

K=[.05,0,100,0,0,1000]
EXP=exp10
process_att $EXP $K $DIR $LOG $HZ

K=[.01,0,100,0,0,1000]
EXP=exp11
process_att $EXP $K $DIR $LOG $HZ

K=[.005,0,100,0,0,0]
EXP=exp12
process_att $EXP $K $DIR $LOG $HZ



DIR=sim
#K=[10,100,0.05,.0025,.0025,0.25]
LOG=exp1
EXP=exp1
HZ=1000

#process_att $EXP $K $DIR $LOG $HZ
