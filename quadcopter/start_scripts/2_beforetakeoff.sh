rosrun mavros mavros_node _fcu_url:=udp://:14550@ _fcu_protocol:=v1.0 &
sleep 2
rostopic hz /mavros/global_position/rel_alt
