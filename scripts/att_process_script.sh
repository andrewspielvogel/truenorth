#!/bin/bash

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p $2/kvh/configs
    mkdir -p $2/kvh/processed
    mkdir -p $2/kvh/pdfs
    
    KVH=$2/$3.KVH
    CSV=$2/kvh/processed/$1.csv
    CONFIG=$2/kvh/configs/$1.m
    PDF=$2/kvh/pdfs/$1.pdf
    PHINS=$2/$3.INS

    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_E_n $8 --k_acc_bias $9 --k_ang_bias ${10} --acc_bias ${11} --ang_bias ${12} --k_g ${13} --k_north ${14}

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}



EXP=exp56
#5

DIR=/log
LOG=2018_07_27
HZ=5000
rpy_align=[-1.57,0,1.57]
rpy_ro=[0,0,-2.2]
k_acc=[.1,.1,.1]
k_acc_bias=[0.05,0,0.05]
k_acc_bias=[0,0,0]
k_ang_bias=[0.0000005,0.000000005,0.0000005]
k_ang_bias=[0,0,0]
k_E_n=[0.0001,0.0000001,0.0001]
k_E_n=[0.000005,0.000005,0.000005]
acc_bias=[-0.00955,0,0.00013]
acc_bias=[0,0,0]
ang_bias=[0.00001466,0,-0.00001454]
ang_bias=[0,0,0]
k_g=[0.05,0.0001,0.05]
k_north=[0.0001,0.0075,0.0001]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north







EXP=exp11
#5

DIR=/log
LOG=2018_07_31_15_09
HZ=5000
rpy_align=[-1.57,0,1.57]
rpy_ro=[0,0,1]
k_acc=[0.1,0.1,0.1]
k_acc_bias=[0.1,0,0.1]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.000001,0.000001,0.000001]
#k_ang_bias=[0,0,0]
k_E_n=[0.0000025,0.0000025,0.0000025]
acc_bias=[-0.01,0,0]
ang_bias=[0,0,0]
k_g=[0.1,0.1,0.1]
k_north=[0.05,0.05,0.05]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north







EXP=exp13
#5

DIR=/log
LOG=2018_08_04
HZ=5000
rpy_align=[-1.5708,0,1.5708]
rpy_ro=[0,0,0]
k_acc=[.1,.1,.1]
k_acc_bias=[0.05,0,0.05]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.000001,0.000001,0.000001]
#k_ang_bias=[0,0,0]
k_E_n=[0.0001,0.00001,0.0001]
#k_E_n=[0.0001,0.0000005,0.0001]
ang_bias=[0.0000035,0.00000269,-0.00001237]
ang_bias=[0,0,0]
acc_bias=[-0.0129,0,-0.003]
acc_bias=[0,0,0]
k_g=[0.1,0.0001,0.1]
k_north=[0.0001,0.05,0.0001]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north


EXP=exp9
#5

DIR=/log
LOG=2018_08_06
HZ=5000
rpy_align=[-1.5708,0,1.5708]
rpy_ro=[0,0,0]
k_acc=[.1,.1,.1]
k_acc_bias=[0.05,0.05,0.05]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.0000005,0.0000005,0.0000005]
#k_ang_bias=[0,0,0]
k_E_n=[0.00005,0.000005,0.00005]
#k_E_n=[0.0001,0.0000005,0.0001]
ang_bias=[0.00001,0.00000269,-0.00001237]
#ang_bias=[0,0,0]
acc_bias=[-0.0129,0,-0.003]
#acc_bias=[0,0,0]
k_g=[0.1,0.0001,0.1]
k_north=[0.0001,0.005,0.0001]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north


EXP=exp31
#28

DIR=/log
LOG=2018_08_06_17-19
HZ=5000
rpy_align=[-1.563176,0.003072,1.57033188]
rpy_ro=[0,0,1]
k_acc=[.1,.1,.1]
k_acc_bias=[0.05,0,0.05]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.000001,0.000001,0.000001]
#k_ang_bias=[0,0,0]
k_E_n=[0.000005,0.000005,0.000005]
#k_E_n=[0.0001,0.0000005,0.0001]
ang_bias=[0.000015,0.00000269,-0.000015]
#ang_bias=[0.00000087,-0.00000349,-0.0000121]
#ang_bias=[0,0,0]
acc_bias=[-0.0129,0,-0.003]
#acc_bias=[-0.0136,-0.0325,0.0533]
#acc_bias=[0,0,0]
k_g=[0.1,0.0001,0.1]
k_north=[0.0001,0.0025,0.0001]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north


EXP=exp20
#28

DIR=/home/spiels/exp/dive2
LOG=2018_08_07
HZ=5000
rpy_align=[-1.5684,0.0016,1.5701]
rpy_ro=[0,0,1]
k_acc=[.1,.1,.1]
k_acc_bias=[0.051,0,0.051]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.000005,0.000005,0.000005]
#k_ang_bias=[0,0,0]
k_E_n=[0.000005,0.000005,0.000005]
ang_bias=[0,0,0]
ang_bias=[0.00000093,0.00000342,-0.00001233]
acc_bias=[0,0,0]
k_g=[0.1,0.0001,0.1]
k_north=[0.0001,0.01,0.0001]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north




EXP=exp1
DIR=/home/spiels/exp/dive3
LOG=2018_08_07
HZ=5000
rpy_align=[-1.5684,0.0016,1.5701]
rpy_ro=[0,0,-0.2]
k_acc=[.1,.1,.1]
k_acc_bias=[0.05,0.05,0.05]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.0000005,0.0000005,0.0000005]
#k_ang_bias=[0,0,0]
k_E_n=[0.0001,0.0001,0.0001]
ang_bias=[0,0,0]
ang_bias=[0.00000005,0.000003,-0.00001]
acc_bias=[-0.01,0,-0.005]
k_g=[0.1,0.0001,0.1]
k_north=[0.0001,0.01,0.0001]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north


EXP=exp3
#14
DIR=/home/spiels/exp/dive3
LOG=2018_08_07
HZ=5000
rpy_align=[-1.5684,0.0016,1.5701]
rpy_ro=[0,0,-0.2]
k_acc=[.1,.1,.1]
k_acc_bias=[0.05,0.05,0.05]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.000001,0.000001,0.000001]
#k_ang_bias=[0,0,0]
k_E_n=[0.000005,0.000005,0.000005]
ang_bias=[0,0,0]
ang_bias=[0,0.000003,-0.00001]
acc_bias=[-0.01,0,-0.005]
k_g=[0.1,0.0001,0.1]
k_north=[0.0001,0.01,0.0001]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north


EXP=exp39
#33
#27
#29
#14
DIR=/home/spiels/exp/dive5
LOG=2018_08_08
HZ=5000
rpy_align=[-1.5684,0.0016,1.5701]
rpy_ro=[0,0,-0.2]
k_acc=[.1,.1,.1]
k_acc_bias=[0.5,0.5,0.5]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.0000025,0.0000025,0.0000025]
#k_ang_bias=[0,0,0]
k_E_n=[0.000005,0.000005,0.000005]
ang_bias=[0,0,0]
ang_bias=[0,0.000003,-0.00001]
acc_bias=[-0.01,0,-0.005]
k_g=[1,1,1]
k_north=[1,1,1]

process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north


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
