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




#K=[0.1,0.005,0.1,0.0001,0,0]
K=[0.01,0.005,0.1,0.0001,0.00001,0.25]
K=[0.01,0.005,0.1,0.0001,0.000001,0.25]
K=[0.01,0.005,0.1,0.0001,0.000001,0]
K=[0.01,0.01,0.01,0.00001,0.0000005,0.05]
K=[0.1,0.01,0.01,0.00001,0,0]

EXP=exp32


DIR=RSS2018
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0.2]


LOG=2018_1_12_10_36
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

LOG=2017_12_21_13_9
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

LOG=2018_1_15_14_37
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


LOG=2018_1_15_15_44
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align




DIR=exp/2018_3_29
LOG=2018_3_29_13_26
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0.1]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


DIR=exp/2018_4_2
LOG=2018_4_2_13_3
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,-0.2]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 
#51


DIR=exp/2018_4_11
LOG=2018_4_11_9_44
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 



LOG=2018_4_11_10_36
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0.2]

process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 



DIR=exp/2018_4_12
LOG=2018_4_12_12_1
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 
