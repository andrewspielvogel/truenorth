#!/bin/sh
# 2018-08-22 RH Created for Truenorth data syncing

# Destination dir of the data
DEST_DIR='/media/data/Sentry08-Data2/2018-sentry-gyro/dives/'

# Source of the data. This will be the sentry kvh stack
SRC_DIR='/media/data/Sentry08-Data1/2018-sentry-gyro/dives/'

RSYNC_CMD="rsync -avh --progress $SRC_DIR $DEST_DIR"
echo $RSYNC_CMD 
$RSYNC_CMD

