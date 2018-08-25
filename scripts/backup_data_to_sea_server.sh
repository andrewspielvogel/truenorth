#!/bin/sh
# 2018-08-22 RH Created for Truenorth data syncing

###########################################
###########################################
#   RUN FROM DATA USER ON GIT SERVER COMPUTER (192.168.100.212)
###########################################
###########################################







# Destination dir of the data
DEST_DIR='/media/data/Sentry08-Data1/2018-sentry-gyro/dives/sentry493/'

# Source of the data. This will be the sentry kvh stack
SRC_IP='192.168.100.112'
SRC_USER='spiels'
SRC_DIR='/log/'

###########################################
# do --dry-run first and check all files
###########################################
# DRY RUN
RSYNC_CMD="rsync -avh --dry-run --progress $SRC_USER@$SRC_IP:$SRC_DIR $DEST_DIR"
# THE REAL THING
# RSYNC_CMD="rsync -avh --progress $SRC_USER@$SRC_IP:$SRC_DIR $DEST_DIR"

echo $RSYNC_CMD 

$RSYNC_CMD

