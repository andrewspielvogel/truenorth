#!/bin/bash
#
# 2018-08-28 LLW For Sentry 493

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p $2/proc/kvh/configs
    mkdir -p $2/proc/kvh/processed
    mkdir -p $2/proc/kvh/pdfs
    
    KVH=$2/kvh/$3.KVH
    CSV=$2/proc/kvh/processed/$1.csv
    CONFIG=$2/proc/kvh/configs/$1.m
    PDF=$2/proc/kvh/pdfs/$1.pdf
    PHINS=$2/phins/$3.INS
    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_E_n $8 --k_acc_bias $9 --k_ang_bias ${10} --acc_bias ${11} --ang_bias ${12} --k_g ${13} --k_north ${14} -l ${15}

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

    evince $PDF &

}

    
DIR=/home/llw/llw/sentry_2018/data/2018-sentry-gyro/dives_10hz/sentry493
#LOG=2018_08_26_07_00
LOG=2018_08_26_0600_to_0830
EXP=${LOG}_exp01
HZ=10
#6
rpy_align=[-1.5798,0.0573,1.5712]
# rpy_ro=[0,0,2.1]
rpy_ro=[0,0,1.2]
k_g=[1,1,1]
k_north=[1,1,1]
# k_north=[10,10,10]
k_acc=[10,10,10]
# k_acc_bias=[5,0,5]
k_acc_bias=[0,0,0]
# k_acc_bias=[0.5,0,0.5]
k_ang_bias=[0,0,0]
# k_ang_bias=[0.000001,0.000001,0.000001]
k_E_n=[0.0001,0.0001,0.0001]
#k_E_n=[0,0,0]
acc_bias=[0.01,0.0,0.0]
# acc_bias=[0.025,0,-0.04]
ang_bias=[0.0,0,0.0]
# values andrew determined from an earlier dive
ang_bias=[0.000003,0,0.00001]
LAT=32.71

#           1    2    3    4   5       6          7      8      9           10          11        12        13   14       15
process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north $LAT
