#!/bin/bash
rosrun mavros mavsys mode -c GUIDED
sleep 3
rostopic echo /mavros/state
