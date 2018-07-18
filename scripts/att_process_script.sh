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
K=[0.01,0.005,0.1,0.0001,0.00000005,0]
K=[1,0.005,0.1,0.0001,0.0000005,0.01]
K=[1,0.005,0.1,0.0001,0.0000001,0.01]
K=[1,0.005,0.1,0.00025,0.0000025,0.025]
#K=[1,0.005,0.1,0.0001,0,0]

K=[0.1,0.025,0.1,0.0001,0,0]
K=[0.1,0.1,0.1,0.0001,0,0]

K=[0.1,0.1,0.1,0.0001,0.000001,0.00001]

EXP=exp66
#53

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
rpy_Ro=[0,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 



DIR=exp/2018_4_12
LOG=2018_4_12_12_1
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0.2]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 




DIR=exp/2018_5_3
LOG=2018_5_3_9_30
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0.4]
EXP=exp34

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 


DIR=exp/2018_5_9
LOG=2018_5_9_9_39
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0.1]
EXP=exp148


#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 


DIR=exp/2018_5_23
LOG=2018_5_23_11_52
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,1.55]
EXP=exp15

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align 


DIR=exp/2018_5_23
LOG=2018_5_23_11_60
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


DIR=exp/2018_5_23
LOG=2018_5_23_12_9
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,-1.55]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

DIR=exp/2018_5_23
LOG=2018_5_23_12_20
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,3.14]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align














#####################################
# static, rot, static test
#####################################

K=[0.1,0.1,0.1,0.0001,0.0000025,0.05]
K=[0.1,0.1,0.1,0.0001,0,0]
K=[0.1,0.1,0.1,0.0001,0.0000025,0.05]


EXP=exp10

DIR=exp/2018_5_30
LOG=2018_5_30_13_50
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


EXP=exp1
K=[0.1,0.1,0.1,0.0001,0,0]

DIR=exp/2018_5_30
LOG=2018_5_30_12_50
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,1.57]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


DIR=exp/2018_5_30
LOG=2018_5_30_13_3
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,3.14]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

DIR=exp/2018_5_30
LOG=2018_5_30_13_17
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align

DIR=exp/2018_5_30
LOG=2018_5_30_13_29
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,-1.57]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align



#####################################
# static, rot, static test
#####################################

K=[0.1,0.1,0.1,0.0001,0.0000025,0.05]
K=[0.1,0.1,0.1,0.0001,0,0]
K=[0.1,0.1,0.01,0.00000001,0,0]
#15
#25
#32
#43-48

DIR=exp/2018_6_1
LOG=2018_6_1_10_57
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,0]


ORD_MAG=10
for w in `seq 1 $ORD_MAG`;
do
    ka=1
    for j in `seq 2 $w`
    do

	ka=0$ka
	
    done
	ka=0.$ka
    for i in `seq 1 $ORD_MAG`;
    do

	UND=_
	EXP=exp_gain_$w$UND$i
	a=$((10**(($i-1))))

	kn=1
	for j in `seq 2 $i`
	do

	    kn=0$kn
	
	done
	kn=0.$kn

	K=[0.1,0.1,$ka,$kn,0,0]

	#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align
	
    done
done

EXP=exp_gain_3_5_bias5
K=[0.1,0.1,0.00001,0.00001,0.0000001,0.01]
#K=[0.1,0.1,0.001,0.001,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


EXP=exp3
K=[0.1,0.1,0.1,0.0001,0,0]

LOG=2018_6_1_10_34
rpy_Ro=[0,0,0.7]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


LOG=2018_6_1_13_8
rpy_Ro=[0,0,0.7]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


K=[0.1,0.1,0.1,0.0001,0.00001,0.5]
K=[0.1,0.1,1,0.00001,0.0001,1]
K=[0.1,0.1,0.001,0.00001,0.0000001,0.05]
K=[0.1,0.1,0.00001,0.00001,0,0]


EXP=exp5

DIR=tn/noise
LOG=exp1
HZ=1000
rpy_align=[0,0,0]
rpy_Ro=[0,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


EXP=exp2

DIR=tn/noise
LOG=exp2
HZ=1000
rpy_align=[0,0,0]
rpy_Ro=[0,0,0.2]

process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align


EXP=exp1

DIR=tn/noise
LOG=exp3
HZ=1000
rpy_align=[0,0,0]
rpy_Ro=[0,0,0]

#process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align
