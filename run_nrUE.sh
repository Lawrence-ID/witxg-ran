#!/bin/sh
cd ./cmake_targets

#40M
#sudo RFSIMULATOR=127.0.0.1 ./ran_build/build/nr-uesoftmodem -r 106 --numerology 1 -C 3619200000 --sa -O ../targets/PROJECTS/GENERIC-NR-5GC/CONF/ue.conf --nokrnmod --rfsim

#10M
sudo RFSIMULATOR=127.0.0.1 ./ran_build/build/nr-uesoftmodem -r 24 --numerology 1 -C 3604320000 -s 24 --sa -O ../targets/PROJECTS/GENERIC-NR-5GC/CONF/ue.conf --nokrnmod --rfsim

