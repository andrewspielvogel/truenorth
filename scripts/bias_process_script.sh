#!/bin/bash

process_bias(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p ~/log/$3/$4/configs
    mkdir -p ~/log/$3/$4/processed
    mkdir -p ~/log/$3/$4/pdfs
    
    KVH=/home/spiels/log/$3/$4.KVH
    CSV=~/log/$3/$4/processed/$1.csv
    CONFIG=~/log/$3/$4/configs/$1.m
    PDF=~/log/$3/$4/pdfs/$1.pdf
    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -k $2 -z $5

    rosrun truenorth mems_process $CONFIG

}



DIR=sim_MEMS/true
K=[0.1,1,1,1,1,0.1,50]
K=[0.1,1,1,1,0.25,0.01,10]
K=[1,1,1,1,0.5,0.05,10]

EXP=exp1
HZ=100
K=[0.05,0.05,1,1,0.1,0.025,0.5]


LOG=exp1
#process_bias $EXP $K $DIR $LOG $HZ

LOG=exp2
#process_bias $EXP $K $DIR $LOG $HZ

LOG=exp3
#process_bias $EXP $K $DIR $LOG $HZ

LOG=exp4
process_bias $EXP $K $DIR $LOG $HZ

LOG=exp5
process_bias $EXP $K $DIR $LOG $HZ

LOG=exp6
#process_bias $EXP $K $DIR $LOG $HZ

K=[0.05,0.05,1,1,0.1,0.025,0.5]
DIR=IROS2018
EXP=exp1
LOG=20180222_1908
process_bias $EXP $K $DIR $LOG $HZ


