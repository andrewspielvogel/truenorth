#!/bin/bash

process_att(){
    
    python ../python/gen_config_file.py -i $1 -o $2 -c $3

    rosrun truenorth post_process $3

    python ../python/plot_att.py -i $2 -o $4

}

KVH=/home/spiels/log/ICRA2018/run1/2017_8_16_11_31.KVH
CSV=~/log/processed/yo.csv
CONFIG=~/log/configs/yo.m
PDF=~/log/pdfs/yo.pdf


process_att $KVH $CSV $CONFIG $PDF
