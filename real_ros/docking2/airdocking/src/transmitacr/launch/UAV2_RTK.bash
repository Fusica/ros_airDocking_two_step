# run by sudo bash UAV1.bash

# sudo chmod 777 /dev/ttyACM0

# cd /home/jetson/airdocking/src/transmitacr/launch

#1. roslaunch /home/jetson/airdocking/src/transmitacr/launch/UAV2mavros.launch

#2.  roslaunch /home/jetson/airdocking/src/transmitacr/launch/sample.launch

#3.  bash /home/jetson/airdocking/src/transmitacr/launch/UAV2.bash

#4. python /home/jetson/airdocking/src/velocity_control/scripts/control_velocity_pid_vrpn.py

#5. python /home/jetson/airdocking/src/velocity_control/scripts/control_velocity_vrpn.py 

#6. python /home/jetson/airdocking/src/transmitacr/script/transmitter.py

#7. bash /home/jetson/real_log/double_motion/bag.bash
rosrun mavros mavcmd -n /child2/mavros long 511 32 450 0 0 0 0 0
rosrun mavros mavcmd -n /child2/mavros long 511 31 450 0 0 0 0 0
rosrun mavros mavcmd -n /child2/mavros long 511 65 900 0 0 0 0 0

rosrun topic_tools relay /vrpn_client_node2/UAV2/pose /child2/mavros/vision_pose/pose


# python /home/jetson/airdocking/src/transmitacr/script/transmitter.py

# python /home/jetson/airdocking/src/velocity_control/scripts/control_velocity_pid_vrpn.py

# python /home/jetson/airdocking/src/velocity_control/scripts/control_velocity_vrpn.py

# cd ~/real_log 
# bash single_bag.bash







# rostopic hz /child2/mavros/local_position/pose

# rostopic echo /child1/mavros/local_position/pose
# rostopic echo /child2/mavros/local_position/pose
# rostopic echo /child2/mavros/state


# rostopic echo /mavros/local_position/pose


# rostopic echo /vrpn_client_node1/UAV1/pose 
# rostopic echo /vrpn_client_node2/UAV2/pose 