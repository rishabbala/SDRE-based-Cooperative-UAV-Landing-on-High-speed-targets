#!/bin/bash
#pass $1 as the same folder argument as bag file command and $2 as dumpfilename according to the date
cd $1 
rosrun test timer_test_vision_vertland_full.py > $2
