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



DIR=sim_MEMS
K=[1,1,1,1,1,0.1,1]
EXP=exp1
HZ=1000


LOG=exp1
process_bias $EXP $K $DIR $LOG $HZ

LOG=exp2
process_bias $EXP $K $DIR $LOG $HZ

LOG=exp3
process_bias $EXP $K $DIR $LOG $HZ
