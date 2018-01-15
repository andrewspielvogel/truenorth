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
    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -k $2 -z $5 -R $6

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $1

}


DIR=RSS2018
HZ=5000


K=[1,.01,1,0.00005,0.0001,0.1]
K=[0.1,0.001,1,0.00005,0.00005,0.25]
K=[0.1,0.1,1.0,0.000025,0.0001,0.5] #this one works well
K=[1,100,0.01,0.0000001,0.00000025,0.5] #thissss converges
#K=[1,10,0.01,0.0000002,0.000001,0.5]
EXP=exp64
EXP=exp144


rpy_Ro=[0,0,0]
LOG=2018_1_12_10_36
LOG=2017_12_21_13_9
process_att $EXP $K $DIR $LOG $HZ $rpy_Ro 


LOG=2017_12_21_14_20
LOG=2018_1_12_10_36
#rpy_Ro=[0,0,1.5]
process_att $EXP $K $DIR $LOG $HZ $rpy_Ro 


DIR=WHOI
LOG=sl29_0_stripped
EXP=exp0
rpy_Ro=[0,0,0]
#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro



DIR=sim
#K=[10,100,0.1,.00001,.001,10]
LOG=exp1
EXP=exp1b
HZ=1000

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro &
