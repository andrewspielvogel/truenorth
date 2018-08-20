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





EXP=exp1
#33
#27
#29
#14
DIR=/home/spiels/exp/dive5/static_post
LOG=2018_08_08_18_39
HZ=5000
rpy_align=[-1.5684,0.0016,1.5701]
rpy_ro=[0,0,0.8]
k_acc=[.1,.1,.1]
k_acc_bias=[0.5,0.5,0.5]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.0000025,0.0000025,0.0000025]
#k_ang_bias=[0,0,0]
k_E_n=[0.000025,0.000025,0.000025]
ang_bias=[0,0,0]
ang_bias=[0,0.000003,-0.00001]
acc_bias=[-0.01,0,-0.005]
k_g=[1,1,1]
k_north=[1,1,1]

#process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north




EXP=exp29
DIR=/home/spiels/exp/dive5
LOG=2018_08_08
HZ=5000
rpy_align=[-1.5684,0.0016,1.5701]
rpy_ro=[0,0,-3.1]
k_acc=[.1,.1,.1]
k_acc_bias=[0.5,0.5,0.5]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.0000025,0.0000025,0.0000025]
#k_ang_bias=[0,0,0]
k_E_n=[0.000025,0.000025,0.000025]
ang_bias=[0,0,0]
acc_bias=[0,0,0]
#ang_bias=[0,0.000003,-0.00001]
#acc_bias=[-0.01,0,-0.005]
k_g=[0.01,0.01,0.01]
k_north=[0.1,0.1,0.1]

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
