#!/bin/bash

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p $2/proc/kvh/configs
    mkdir -p $2/proc/kvh/processed
    mkdir -p $2/proc/kvh/pdfs
    
    KVH=$2/kvh/$3_dvl.KVH
    CSV=$2/proc/kvh/processed/$1.csv
    CONFIG=$2/proc/kvh/configs/$1.m
    PDF=$2/proc/kvh/pdfs/$1.pdf
    PHINS=$2/phins/$3.INS
    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_E_n $8 --k_acc_bias $9 --k_ang_bias ${10} --acc_bias ${11} --ang_bias ${12} --k_g ${13} --k_north ${14} -l ${15}

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}



DIR=/home/lcsr/cruise_data/dives_10hz/sentry494
LOG=dive_10hz_bottom
EXP=${LOG}_exp11
#7
HZ=10
rpy_align=[-1.5798,0.0573,1.5712]
rpy_ro=[0,0,-2.2]
k_g=[1,1,1]
k_north=[0.001,0.001,0.001]
k_acc=[10,10,10]
k_acc_bias=[0.25,0,0.25]
k_acc_bias=[0,0,0]
k_ang_bias=[0.000001,0.000001,0.000001]
#k_ang_bias=[0,0,0]
k_E_n=[0.0005,0.0005,0.0005]
#k_E_n=[0,0,0]
acc_bias=[0.07,0,-0.02]
acc_bias=[0,0,0]
ang_bias=[0.000002,0,0.000013]
#ang_bias=[0,0,0]
LAT=32.71

process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north $LAT
