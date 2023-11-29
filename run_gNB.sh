#!/bin/sh
cd ./cmake_targets

# 40M
#sudo ./ran_build/build/nr-softmodem --sa -O ../targets/PROJECTS/GENERIC-NR-5GC/CONF/gnb.sa.band78.fr1.106PRB.oxgrf.conf --rfsim

#10M
sudo ./ran_build/build/nr-softmodem --sa -O ../targets/PROJECTS/GENERIC-NR-5GC/CONF/gnb.sa.band78.fr1.24PRB.usrpb210.conf --rfsim
