#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_


#include <tf/tf.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <Eigen/Eigen>
#include <vector>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>


using namespace std;
using namespace Eigen;


class TRAJECTORY_GENERATOR{
private:

    // Param from launch file
    double _vis_traj_width;
    double _Vel, _Acc;
    int _dev_order, _min_order;

    int _poly_num1D;
    MatrixXd _polyCoeff;
    VectorXd _polyTime;

    Vector3d _startPos  = Vector3d::Zero();
    Vector3d _startVel  = Vector3d::Zero();
    Vector3d _startAcc  = Vector3d::Zero();
    Vector3d _endVel    = Vector3d::Zero();
    Vector3d _endAcc    = Vector3d::Zero();


    ros::Subscriber waypoints_sub;
    ros::Publisher path_vis_pub, trajectory_vis_pub;

    ros::Publisher _path_vis_pub;


    //路标回调函数
    void waypointsCallback(const nav_msgs::Path::ConstPtr &msg);

    //轨迹生成函数
    void trajGeneration(Eigen::MatrixXd& path);

    //时间分配,为每一段轨迹合理分配时间
    VectorXd timeAllocation(Eigen::MatrixXd& Path);

    //路标点可视化
    void visWayPointPath(Eigen::MatrixXd& path);

    //轨迹可视化
    void visTrajectory(Eigen::MatrixXd& polyCoeff, Eigen::VectorXd& time);

    //计算多项式轨迹在给定时间参数t处的三维坐标
    Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t);

   

public:
    TRAJECTORY_GENERATOR(/* argvs */){

    }
    ~TRAJECTORY_GENERATOR(){

    }
    void init(ros::NodeHandle& nh);
};




class TRAJECTORY_GENERATOR_WAYPOINT  
{
private:
    // double _qp_cost;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _M;
    Eigen::MatrixXd _Ct;
    
    Eigen::VectorXd _Px, _Py, _Pz;

    Eigen::MatrixXd getQ(const int p_num1d,
                         const int order, 
                         const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getM(const int p_num1d,
                         const int order, 
                         const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getCt(const int seg_num, const int d_order);

    Eigen::VectorXd closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                         const Eigen::MatrixXd &M,
                                         const Eigen::MatrixXd &Ct,
                                         const Eigen::VectorXd &WayPoints1D,
                                         const Eigen::VectorXd &StartState1D,
                                         const Eigen::VectorXd &EndState1D,
                                         const int seg_num, 
                                         const int d_order);

public:
    TRAJECTORY_GENERATOR_WAYPOINT();

    ~TRAJECTORY_GENERATOR_WAYPOINT();

    Eigen::MatrixXd PolyQPGeneration(
        const int order,
        const Eigen::MatrixXd &Path,
        const Eigen::MatrixXd &Vel,
        const Eigen::MatrixXd &Acc,
        const Eigen::VectorXd &Time);

    int Factorial(int x);
};



#endif




