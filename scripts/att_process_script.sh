#!/bin/bash

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p $2/proc/kvh/configs
    mkdir -p $2/proc/kvh/processed
    mkdir -p $2/proc/kvh/pdfs
    
    KVH=$2/kvh/acc$3avg500.KVH
    CSV=$2/proc/kvh/processed/$1.csv
    CONFIG=$2/proc/kvh/configs/$1.m
    PDF=$2/proc/kvh/pdfs/$1.pdf
    PHINS=$2/phins/$3.INS

    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_E_n $8 --k_acc_bias $9 --k_ang_bias ${10} --acc_bias ${11} --ang_bias ${12} --k_g ${13} --k_north ${14}

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}


DIR=/home/lcsr/cruise_data/sentry491
LOG=2018_08_23_22-23
LOG=test
EXP=${LOG}_exp13
HZ=5000
rpy_align=[-1.5798,0.0573,1.5712]
rpy_ro=[0,0,1]
k_acc=[.1,.1,.1]
k_acc_bias=[0.25,0,0.25]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.0000005,0.0000005,0.0000005]
#k_ang_bias=[0,0,0]
k_E_n=[0.000001,0.000001,0.000001]
ang_bias=[0,0,0]
acc_bias=[0,0,0]
#ang_bias=[0.00001,0,0.000003]
#acc_bias=[0.07,0,-0.05]
k_g=[0.01,0.01,0.01]
k_north=[0.1,0.1,0.1]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north



DIR=/home/lcsr/cruise_data/sentry491
LOG=2018_08_24_05-07
EXP=${LOG}_exp24
rpy_ro=[0,0,2.3]
k_acc_bias=[0.01,0.01,0.01]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.00000025,0.00000025,0.00000025]
#k_ang_bias=[0,0,0]
k_E_n=[0.000005,0.000005,0.000005]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north


DIR=/home/lcsr/cruise_data/sentry492
LOG=2018_08_25_01_00
EXP=${LOG}_exp49
#26
#13 add mag feedback
rpy_ro=[0,0,2.5]
k_g=[0.1,0.1,0.1]
k_north=[0,0,0]
k_acc=[0.1,0.1,0.1]
k_acc_bias=[1,0,1]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.0000001,0.0000001,0.00000001]
k_ang_bias=[0,0,0]
k_E_n=[0.00002,0.00002,0.00002]
k_E_n=[0,0,0]
#ang_bias=[0.000001,0,0]
acc_bias=[0,0,0]
#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north


DIR=/home/lcsr/cruise_data/sentry491
#LOG=2018_08_26_07_00
LOG=2018_08_24_05-07
EXP=${LOG}_exp5
HZ=10
#6
rpy_ro=[0,0,2.1]
k_g=[1,1,1]
k_north=[1,1,1]
k_acc=[10,10,10]
k_acc_bias=[5,0,5]
k_acc_bias=[0,0,0]
k_ang_bias=[0.000001,0.000001,0.000001]
k_ang_bias=[0,0,0]
k_E_n=[0.0001,0.0001,0.0001]
#k_E_n=[0,0,0]
process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north
