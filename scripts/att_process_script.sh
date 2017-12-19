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
K=[1,0,.05,.00,0.00,.1]


LOG=2017_12_13_15_16
EXP=exp1
HZ=5000

process_att $EXP $K $DIR $LOG $HZ



LOG=2017_12_13_15_59
EXP=exp2


process_att $EXP $K $DIR $LOG $HZ

DIR=sim
#K=[10,100,0.05,.0025,.0025,0.25]
LOG=exp1
EXP=exp1
HZ=1000

process_att $EXP $K $DIR $LOG $HZ
