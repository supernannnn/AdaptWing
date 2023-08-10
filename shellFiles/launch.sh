#! /bin/bash
sudo chmod 777 /dev/ttyACM0 & sleep 3;
sudo chmod 777 /dev/ttyUSB0 & sleep 2;
roslaunch mavros px4.launch & sleep 5;
roslaunch realsense2_camera rs_camera.launch & sleep 10;
# roslaunch vins AdaptWing.launch
roslaunch vins AdaptWing.launch & sleep 3;
roslaunch odom_fusion odom_fusion.launch & sleep 3;
roslaunch usb_cam usb_cam-test.launch & sleep 3;
roslaunch apriltag_ros continuous_detection.launch
wait;
