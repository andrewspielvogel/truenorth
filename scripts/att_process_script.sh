#!/bin/bash

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p ~/log/$3/$4/configs
    mkdir -p ~/log/$3/$4/processed
    mkdir -p ~/log/$3/$4/pdfs
    
    KVH=/home/spiels/log/$3/$4.KVH
    CSV=~/log/$3/$4/processed/$1.csv
    CONFIG=~/log/$3/$4/configs/$1.m
    PDF=~/log/$3/$4/pdfs/$1.pdf
    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -k $2 -z $5 -R $6 -a $7

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $4 #$1

}


DIR=RSS2018
HZ=5000



K=[0.1,0.1,1.0,0.000025,0.0001,0.5] #this one works well
K=[1,100,0.01,0.0000001,0.00000025,0.1] #thissss converges
K=[0.1,0.1,0.01,0.000005,0.000005,0.1]
#K=[0.1,0.1,1,0.0001,0.0001,0.1]
#K=[0.1,0.1,1,0.0001,0.0001,0.1]
K=[0.1,0.1,0.01,0.0000001,0.0000001,0.5] # save
#K=[0.1,0.001,0.01, 0.000001,0.000001,1] #the one working across sim and exp
K=[0.1,1,0.01,0.000001,0.000001,1]
rpy_align=[1.5708,0,-1.5708]
EXP=exp66


rpy_Ro=[0,0,0.2]


LOG=2018_1_12_10_36
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

LOG=2017_12_21_13_9
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

LOG=2018_1_15_14_37
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


LOG=2018_1_15_15_44
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align




DIR=tn/true
K=[1,0.005,1,0.001,0.00005,0.5]
K=[1,0.01,1,0.001,0.00001,0.5]

EXP=exp1
HZ=1000
rpy_align=[0,0,0]
rpy_Ro=[0.2,-0.1,0.3]



LOG=exp1
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


LOG=exp2
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

DIR=tn/noise
EXP=exp9



LOG=exp2
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

LOG=exp4
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

LOG=exp5
process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align
