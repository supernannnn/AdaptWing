#ifndef _CONSOLE_HPP_
#define _CONSOLE_HPP_

#include <tf/tf.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <limits>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <Eigen/Eigen>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <visual_utils/planning_visualization.hpp>
#include <tf/transform_broadcaster.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

//自定义的控制台状态消息
#include <console/ConsoleState.h>

#include <circle_detection.h>



using namespace std;
using namespace Eigen;


/*线速度控制状态*/
enum VEL_STATE {
    FORWARD = 0,   //前进
    MOVESIDE        //左右
};



/*任务状态*/
enum STATE{
    INIT = 0,                       //初始化
    WAIT_TAKE_OFF_FINISHED ,        //等待起飞完成              
    FLY_TO_STRAT,                   //飞向起点
    SEARCHING_PILLAR,               //搜寻第一根柱子
    ROTATE_PILLARS,                 //执行绕柱轨迹
    FLYING_VIA_TUNNEL,              //钻隧道
    SEARCHING_MAZE,                 //迷宫
    PASSING_CIRCLES,                //穿圆
    SEARCHING_APRILTAG,             //搜寻二维码
    LANDING                         //降落
};


struct MiniSnapTraj{
    ros::Time       start_time;         //轨迹起始时间
    int             traj_nums;          //分段轨迹数量
    double          time_duration;      //轨迹总时间
    Eigen::VectorXd poly_time;          //每段轨迹的结束时间点，相对轨迹起始点来说
    Eigen::MatrixXd poly_coeff;         //多项式轨迹系数矩阵
    MiniSnapTraj(){};
    MiniSnapTraj(ros::Time _start_time, Eigen::VectorXd _poly_time, Eigen::MatrixXd _poly_coeff){
        start_time  = _start_time;
        traj_nums   = _poly_time.size();
        poly_time.resize(traj_nums);
        double sum = 0;
        for (int i = 0; i < _poly_time.size(); i++){
            sum += _poly_time(i);
            poly_time[i] = sum;
        }
        poly_coeff  = _poly_coeff;
        for (int i = 0; i < _poly_time.size(); i++)  time_duration += _poly_time(i);
    };
    ~MiniSnapTraj(){};
};


class CONSOLE{
private:


    /***********************************minimumsnap***********************************/
    double _vis_traj_width;
    double _Vel, _Acc;
    int _dev_order, _min_order;
    int _poly_num1D;

    Vector3d _startPos  = Vector3d::Zero();
    Vector3d _startVel  = Vector3d::Zero();
    Vector3d _startAcc  = Vector3d::Zero();
    Vector3d _endVel    = Vector3d::Zero();
    Vector3d _endAcc    = Vector3d::Zero();

    //路标可视化
    void waypointsVisual(const Eigen::MatrixXd& path);
    //轨迹生成函数
    MiniSnapTraj trajGeneration(Eigen::MatrixXd& path);
    //时间分配,为每一段轨迹合理分配时间
    Eigen::VectorXd timeAllocation(Eigen::MatrixXd& Path);
    //计算多项式轨迹在给定时间参数t处的位置与速度
    std::pair<Eigen::Vector3d, Eigen::Vector3d>  getTrajInfo(const MiniSnapTraj& traj, double t);
    //根据多项式系数和时间分配得到三维坐标点
    void getVisual(MiniSnapTraj& trajectory);
    //轨迹三维坐标点得到轨迹每个点的偏航角
    std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last, MiniSnapTraj& trajectory);
    
    /***********************************minimumsnap***********************************/


    //第一关柱子坐标
    Eigen::Vector3d pillar1    = Vector3d::Zero();
    Eigen::Vector3d pillar2    = Vector3d::Zero();
    Eigen::Vector3d pillar3    = Vector3d::Zero();
    Eigen::Vector3d pillar4    = Vector3d::Zero();

    //第二关隧道起点和终点
    Eigen::Vector3d tunnel_start    = Vector3d::Zero();
    Eigen::Vector3d tunnel_end      = Vector3d::Zero();

    //第三关迷宫路标点
    Eigen::Vector3d maze_wp1    = Vector3d::Zero();
    Eigen::Vector3d maze_wp2    = Vector3d::Zero();

    //第四关圆预设终点

    Eigen::Vector3d circle_terminal = Vector3d::Zero();


    //终点预设坐标
    Eigen::Vector3d terminal_apriltag = Vector3d::Zero();


    ros::Timer ControlCmdCallback_Timer;
    ros::Timer Transformer_Timer;
    ros::Timer FSM_Timer;
    tf::TransformBroadcaster broadcaster;

    /*状态机当前状态*/
    int state;

    int vel_state = 2;

    //从起飞点到第一个预设点的轨迹
    MiniSnapTraj Traj_start;

    //绕柱子轨迹
    MiniSnapTraj rotate_pillars_traj;

    //绕完柱子飞向隧道起点轨迹
    MiniSnapTraj fly2tunnel_start_traj;
    bool fly2tunnel_finished = false;

    //隧道轨迹
    MiniSnapTraj tunnel_traj;
    bool tunnel_finished = false;



    //飞向预设终点轨迹
    MiniSnapTraj fly2apriltag_traj;

    //飞向检测到的二维码
    MiniSnapTraj fly2land_traj;



    //圆检测接口
    CIRCLE::Ptr circle_dec;
    std::pair<bool, cv::Point3d> circle_pos;



    vector<Eigen::Vector3d> traj, start_traj;
    
    PlanningVisualization::Ptr trajVisual_;
    nav_msgs::Path trajPath;
    ros::Publisher trajPath_pub;
    ros::Publisher controlCmd_pub;
    ros::Publisher vel_cmd_pub;
    ros::Publisher land_pub;
    ros::Publisher ego_planner_wps_pub;
    ros::Publisher console_state_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber ego_planner_cmd_sub;
    ros::Subscriber apriltag_sub;
    ros::Subscriber color_sub;
    

    Eigen::Quaterniond odom_q;
    Eigen::Vector3d odom_pos, odom_vel, pillars_terminal_position, apriltag_pos;

    bool has_odom;
    bool rotate_pillars_finished, start_trajectory_finished, eight_follow_finished;
    int tra_id;
    quadrotor_msgs::PositionCommand posiCmd;

    bool is_complete_Start_Traj;
    double last_yaw_, last_yaw_dot_;
    double time_forward_;
    
    bool is_land;
    bool pillar_detected;

    bool apriltag_detected;

    bool have_depth;

    //飞向预设终点是否完成
    bool fly2apriltag_finished;

    //飞向检测Apriltag是否完成
    bool fly2land_finished;

    cv::Mat depth_image;

    bool have_circle = false;
    bool passed_circle_flag = false;

    double circle_dis = 0.0;

    double circle_D = 0.0;

    int have_circle_cnt = 0;

    void CmdForwardCallback(const quadrotor_msgs::PositionCommandPtr msg);

    void ApriltagCallback(const apriltag_ros::AprilTagDetectionArrayPtr& msg);
    void ControlCmdCallback(const ros::TimerEvent &e);
    void OdomCallback(const nav_msgs::OdometryConstPtr msg);
    void DepthCallback(const sensor_msgs::Image::ConstPtr& msg);
    /*TF树坐标变换关系*/
    void TransformerCallback(const ros::TimerEvent &e);
    /*控制状态机*/
    void ControlFSM(const ros::TimerEvent &e);
    /*根据轨迹解析控制指令*/
    void ControlParse(MiniSnapTraj& trajectory, ros::Time start_time, bool init, bool& finished, bool isAdjustYaw);
    std::pair<bool, double> findPillar();
    inline double Distance_of_waypoints(Eigen::Vector3d& Point1, Eigen::Vector3d& Point2);

    /*ego-planner：迷宫路标点生成*/
    vector<geometry_msgs::Point> Ego_planner_mazeWps_genaration();

    void ColorImageCallback(const sensor_msgs::Image::ConstPtr& msg);

    //机体系左右移动函数
    void BodyMoveControlParse(Eigen::Vector3d& ori);

    //机体系前进函数
    //void BodyForwardControlParse(Eigen::Vector3d& ori, double y);



public:
    CONSOLE(/* argvs */){
        has_odom = false;
        is_complete_Start_Traj = false;
        state = INIT;
        rotate_pillars_finished = false;
        start_trajectory_finished = false;
        is_land = false;

        tra_id = 0;
        pillar_detected = false;

        apriltag_detected = false;

        have_depth = false;

        fly2apriltag_finished = false;

        fly2land_finished = false;
    }
    ~CONSOLE(){

    }
    void init(ros::NodeHandle& nh);
};

inline double CONSOLE::Distance_of_waypoints(Eigen::Vector3d& Point1, Eigen::Vector3d& Point2) {
    double x_diff = Point2.x() - Point1.x();
    double y_diff = Point2.y() - Point1.y();
    double z_diff = Point2.z() - Point1.z();
    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}


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




