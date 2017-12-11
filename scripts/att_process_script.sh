#!/bin/bash

process_att(){
    
    python ../python/gen_config_file.py -i $1 -o $2 -c $3 -k $6

    rosrun truenorth post_process $3

    python ../python/plot_att.py -i $2 -o $4 -e $5

    gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/default -dNOPAUSE -dQUIET -dBATCH -dDetectDuplicateImages -dCompressFonts=true -r2 -sOutputFile=/home/spiels/log/pdfs/comp_$5.pdf /home/spiels/log/pdfs/$5.pdf

}

DIR=RSS
LOG=2017_11_17_15_53
EXP=exp2
K=[1,100,0.01,0.0001,0.00000,0.15]


KVH=/home/spiels/log/$DIR/$LOG.KVH
CSV=~/log/processed/$EXP.csv
CONFIG=~/log/configs/$EXP.m
PDF=~/log/pdfs/$EXP.pdf


process_att $KVH $CSV $CONFIG $PDF $EXP $K

