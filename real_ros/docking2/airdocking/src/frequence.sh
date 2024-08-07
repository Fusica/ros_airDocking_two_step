rosrun mavros mavcmd long 511 31 450 0 0 0 0 0 
rosrun mavros mavcmd long 511 32 450 0 0 0 0 0
rosrun mavros mavcmd long 511 65 900 0 0 0 0 0   
rostopic hz /mavros/local_position/pose
