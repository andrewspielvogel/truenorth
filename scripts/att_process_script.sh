#!/bin/bash

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p $3/$4/configs
    mkdir -p $3/$4/processed
    mkdir -p $3/$4/pdfs
    
    KVH=$3/$4.KVH
    CSV=$3/$4/processed/$1.csv
    CONFIG=$3/$4/configs/$1.m
    PDF=$3/$4/pdfs/$1.pdf
    
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -k $2 -z $5 -R $6 -a $7

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $4 #$1

}



K=[0.1,0.1,0.1,0.0001,0,0]
K=[0.1,0.1,0.1,0.0001,0.0000025,0.05]



EXP=exp1

DIR=/log/kvh
LOG=2018_07_18_13_26
HZ=5000
rpy_align=[1.5708,0,-1.5708]
rpy_Ro=[0,0,1.57]

process_att $EXP $K $DIR $LOG $HZ $rpy_Ro $rpy_align
























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
