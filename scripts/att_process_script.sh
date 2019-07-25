#!/bin/bash

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

    echo $KVH
    echo $PHINS
    echo $CSV
    python $TRUENORTH/python/gen_config_file.py -i $KVH -o $CSV -c $CONFIG -z $4 --rpy_ro $5 --rpy_align $6 --k_acc $7 --k_E_n $8 --k_acc_bias $9 --k_ang_bias ${10} --acc_bias ${11} --ang_bias ${12} --k_g ${13} --k_north ${14} -l ${15}

    rosrun truenorth post_process $CONFIG

    python $TRUENORTH/python/plot_att.py -i $CSV -o $PDF -e $3 -p $PHINS

}



DIR=/home/spiels/log/JHUROV/dives_10hz/JHUROV
#DIR=/home/spiels/log/dives_10hz/JHUROV_DVL
#DIR=/home/spiels/log/JHUROV/

LOG=dive10-08
#LOG=2018_12_05_18_17
#LOG=2018_12_14_16_37
#DIR=/home/spiels/exp/dive2
#LOG=2018_08_07
#DIR=/home/spiels/2018-Sentry/cruise_data/dives_10hz/sentry494
#LOG=dive_10hz_bottom
DIR=/home/spiels/log/sim ## comment this
LOG=sim2 ## comment this
EXP=${LOG}_exp6
HZ=1000
#HZ=5000
HZ=10
rpy_align=[-1.5708,0,1.5708]
rpy_align=[-1.5676,0.0021,1.577]

#rpy_align=[-1.5798,0.0573,1.5712]
rpy_align=[0,0,0] ## comment this
rpy_ro=[0,0,0.24]
k_g=[1,1,1]
k_north=[1,1,1]
k_acc=[0.2,0.2,0.2]
k_acc_bias=[1,1,1]
k_ang_bias=[0.00001,0.00001,0.00001]
k_E_n=[0.001,0.001,0.001]
#k_E_n=[0,0,0]
acc_bias=[-0.012,0,-0.003]
ang_bias=[0.000003,0,-0.000015]
ang_bias=[0,0,0]
acc_bias=[0,0,0]
#k_ang_bias=[0,0,0]
#k_acc_bias=[0,0,0]
#k_ang_bias=[0,0,0]
#k_E_n=[0,0,0]





#optimization using the following as initial configs using rms after 10 mins rms and exp6 where it is 10hz and w=[0;0;cos(t/30)/5] 
#startup
#acc=0.2
#east=0.001
#bang=0.00001
#bacc=1


# optimized gains -- run on rov exp using rms after 20 mins
#startup
acc=0.27
east=0.00096
bang=0.000018
bacc=0.80
#rpy_ro=[0,0,0]


acc=0.55
east=0.0015
bang=0.000016
bacc=0.74
#bacc=0
#bang=0
#rpy_ro=[0,0,1.3]

g=1
north=.1



k_acc=[$acc,$acc,$acc]
k_E_n=[$east,$east,$east]
k_ang_bias=[$bang,$bang,$bang]
k_acc_bias=[$bacc,$bacc,$bacc]
k_g=[$g,$g,$g]
k_north=[$north,$north,$north]

LAT=39.32

process_att $EXP $DIR $LOG $HZ $rpy_ro $rpy_align $k_acc $k_E_n $k_acc_bias $k_ang_bias $acc_bias $ang_bias $k_g $k_north $LAT

