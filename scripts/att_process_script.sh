#!/bin/bash

process_att(){
    
    python ../python/gen_config_file.py -i $1 -o $2 -c $3 -k $6

    rosrun truenorth post_process $3

    python ../python/plot_att.py -i $2 -o $4 -e $5

}

DIR=RSS
LOG=2017_11_17_15_53
EXP=exp2_new_gain
K=[100,1000,0.01,0.00001,0.0000,0.025]


KVH=/home/spiels/log/$DIR/$LOG.KVH
CSV=~/log/processed/$EXP.csv
CONFIG=~/log/configs/$EXP.m
PDF=~/log/pdfs/$EXP.pdf


process_att $KVH $CSV $CONFIG $PDF $EXP $K

evince /home/spiels/log/pdfs/$EXP.pdf
