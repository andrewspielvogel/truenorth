#!/bin/bash

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p $2/kvh/$3/configs
    mkdir -p $2/kvh/$3/processed
    mkdir -p $2/kvh/$3/pdfs
    
    KVH=$2/kvh/$3.KVH
    CSV=$2/kvh/$3/processed/$1.csv
    CONFIG=$2/kvh/$3/configs/$1.m
    PDF=$2/kvh/$3/pdfs/$1.pdf
    PHINS=$2/phins/$3.INS

    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_E_n $8 --k_acc_bias $9 --k_ang_bias ${10} --acc_bias ${11} --ang_bias ${12} --k_g ${13} --k_north ${14}

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}



EXP=exp21

DIR=/log
LOG=2018_07_23_16_00
HZ=5000
rpy_align=[-1.57,0,1.57]
rpy_ro=[0,0,-2.2]
k_acc=[0.1,0.1,0.1]
k_acc_bias=[0.05,0.0,0.05]
#k_acc_bias=[0,0,0]
k_ang_bias=[0.00005,0.00005,0.00005]
#k_ang_bias=[0,0,0]
k_E_n=[0.0000025,0.0000025,0.0000025]
acc_bias=[0,0,0]
ang_bias=[0,0,0]
k_g=[0.1,0.1,0.1]
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
