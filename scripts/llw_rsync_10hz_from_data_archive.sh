#!/bin/bash
#
# 2018-08-28 LLW Created to 

echo Running dry-run command: rsync -avh --progress --dry-run  data@192.168.100.212:/media/data/Sentry08-Data1/2018-sentry-gyro/dives_10hz /home/llw/llw/sentry_2018/data/2018-sentry-gyro
rsync -avh --progress --dry-run  data@192.168.100.212:/media/data/Sentry08-Data1/2018-sentry-gyro/dives_10hz /home/llw/llw/sentry_2018/data/2018-sentry-gyro

read -p "Press enter to continue"

echo Running command: rsync -avh --progress data@192.168.100.212:/media/data/Sentry08-Data1/2018-sentry-gyro/dives_10hz /home/llw/llw/sentry_2018/data/2018-sentry-gyro
rsync -avh --progress data@192.168.100.212:/media/data/Sentry08-Data1/2018-sentry-gyro/dives_10hz /home/llw/llw/sentry_2018/data/2018-sentry-gyro
