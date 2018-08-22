#!/bin/sh
# 2018-08-22 RH Created for Truenorth data syncing

# Destination dir of the data
DEST_DIR='/media/data/Sentry08-Data1/2018-sentry-gyro/dives/test-dive-2/'

# Source of the data. This will be the sentry kvh stack
SRC_IP='192.168.100.112'
SRC_USER='spiels'
SRC_DIR='/log/'

RSYNC_CMD="rsync -avh --progress $SRC_USER@$SRC_IP:$SRC_DIR $DEST_DIR"
echo $RSYNC_CMD 

$RSYNC_CMD

