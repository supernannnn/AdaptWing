#ifndef __LOCATION_H
#define __LOCATION_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>

#include <thread>
#include <serial/serial.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>

#include <vector>


// typdef struct RTK_DATA{
//     double float Latitude;
//     double float Longitude;
//     double float Altitude;
// };


class LOCATION{
private:

    ros::Timer processData_timer_;

    nav_msgs::Odometry          odom_msg_;
    geometry_msgs::Quaternion   ori_msg_;

    geometry_msgs::Vector3      angular_velocity;
    geometry_msgs::Vector3      linear_velocity;

    //用于发布里程计信息
    ros::Publisher odom_pub;



    //订阅
    ros::Subscriber baro_alti_sub, imu_data_sub, linear_vel_sub;


    /*
    **RTK模块的串口参数设置
    */
    void read_rtk_via_uart();


    /*
    **用于解析RTK串口读回的gpgga数据流
    */
    void parse_gpgga(const std::string& gpgga_data);



    /*
    **激光测距（TFmini Plus）模块的串口参数设置及数据解析
    */
    void read_laser_via_uart();

    /*
    **完成所有传感器数据的融合，并发布odometry
    */
    void processDataCallback(const ros::TimerEvent &e);
    

    void processBaroCallback(const mavros_msgs::Altitude::ConstPtr &msg);
    void processImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void processLinearVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);


public:

    float laser_altitude;

    //构造函数可以传递参数进来
    LOCATION(/* argvs */){

    }
    ~LOCATION(){

    }

    void init(ros::NodeHandle& nh);
};

#endif