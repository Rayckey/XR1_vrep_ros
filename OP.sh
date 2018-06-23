#!/bin/bash  
echo "Launching Stuff"  
roscore &
echo "Core started" 
rqt --standalone vrep_test &
cd V-REP
./vrep.sh scenes/optimizationprimeclosed_liang.ttt  
wait 
cd ..
ps -A |grep roscore |grep -v grep|cut -c 9-15|xargs kill -1  
