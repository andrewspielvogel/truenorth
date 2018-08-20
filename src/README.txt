Directions for checking hex logging

KVH


cat ~/exp/dive5/2018_08_08_15_45.BKVH | ./hex2bin FE81 > kvh.binary

Dump kvh.binary out serial port using cutecom and parse with kvh node:
roslaunch truenorth att_est.launch

head -n 1000 /log/kvh/2018_08_20_00_38.KVH | cut --bytes=71- | cut --bytes=-364 > hex_processed.ascii
head -n 1000 ~/exp/dive5/2018_08_08_15_45.KVH | cut --bytes=71- | cut --bytes=-364 > orig.ascii
diff orig.ascii hex_processed.ascii





PHINS

cat ~/exp/dive5/2018_08_08_15_45.BINS | ./hex2bin 0090 > phins.binary

Dump to udp with:
cat phins.binary | ./throttle 24 | socat - udp-sendto:127.0.0.1:23454

and parse with phins node:
roslaunch phins phins.launch

head -n 100 /log/phins/2018_08_20_13_10.INS | cut --bytes=89- > phins_processed.ascii
head -n 100 exp/dive5/2018_08_08_15_45.INS | cut --bytes=89- > phins_orig.ascii
