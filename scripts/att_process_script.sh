#!/bin/bash

process_att(){

    KVH=/home/spiels/log/$3/$4.KVH
    CSV=~/log/processed/$1.csv
    CONFIG=~/log/configs/$1.m
    PDF=~/log/pdfs/$1.pdf
    
    python ../python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -k $2

    rosrun truenorth post_process $CONFIG

    python ../python/plot_att.py -i $CSV -o $PDF -e $1

}


DIR=RSS
K=[0.5,100,.1,.01,0.001,.1]


LOG=2017_12_13_15_16
EXP=exp1

process_att $EXP $K $DIR $LOG



LOG=2017_12_13_15_59
EXP=exp2


process_att $EXP $K $DIR $LOG


