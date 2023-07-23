sudo chmod +x /dev/ttyACM0 & sleep 2;
roslaunch mavros px4.launch & sleep 5;
rosrun rtk location_node;
wait;
