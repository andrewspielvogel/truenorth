#!/bin/bash

process_att(){

    TRUENORTH="$(rospack find truenorth)"

    mkdir -p $2/proc/kvh/configs
    mkdir -p $2/proc/kvh/processed
    mkdir -p $2/proc/kvh/pdfs

    #file locations
    KVH=$2/kvh_10hz/$3-10hz.KVH
    CSV=$2/proc/kvh/processed/$1.csv
    CONFIG=$2/proc/kvh/configs/$1.m
    PDF=$2/proc/kvh/pdfs/$1.pdf
    PHINS=$2/phins/$3.INS

    echo $KVH
    echo $PHINS
    echo $CSV

    # generate config file for post_processing code
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_E_n $8 --k_acc_bias $9 --k_ang_bias ${10} --acc_bias ${11} --ang_bias ${12} --k_g ${13} --k_north ${14} -l ${15}

    # run truenorth post processing code
    rosrun truenorth post_process $CONFIG

    # generate plot pdf
    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}


# directory where your kvh/ phins/ proc/ directories are
DIR=/home/spiels/log/KVH_DVL

# log file name
LOG=2019_09_12_16_00

# output file appendix
EXP=${LOG}_exp1

# kvh sample rate
HZ=1000
HZ=10

# kvh to phins alignment [r,p,h]
rpy_align=[-1.5708,0,1.5708]
#rpy_align=[-1.5727,0,1.5224]

# attitude initial condition
rpy_ro=[0,0,0.7]

# sensor bias initial conditions
ang_bias=[0,0,0]
acc_bias=[0,0,0]
ang_bias=[0.000056,0,0.000017]
acc_bias=[-0.023,0,-0.026]


# sensor bias observer gains
acc=0.55
east=0.0015
bang=0.000016
bacc=0.74


acc=.1
east=0.001
east=0.0005
bang=0.0000025
bacc=0.1

# attitude observer gains
g=1
north=1


k_acc=[$acc,$acc,$acc]
k_E_n=[$east,$east,$east]
k_ang_bias=[$bang,$bang,$bang]
k_acc_bias=[$bacc,$bacc,$bacc]
k_g=[$g,$g,$g]
k_north=[$north,$north,$north]

LAT=39.32

process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north $LAT

