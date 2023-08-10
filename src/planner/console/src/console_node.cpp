#include "console/console.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "console_node");
    ros::NodeHandle nh("~");
    CONSOLE console;
    console.init(nh);
    ros::spin();
    return 0;
}

void CONSOLE::init(ros::NodeHandle& nh){
    nh.param("planning/vel", _Vel, 1.0);
    nh.param("planning/acc", _Acc, 1.0);
    nh.param("planning/dev_order", _dev_order, 3);  // the order of derivative, _dev_order = 3->minimum jerk, _dev_order = 4->minimum snap
    nh.param("planning/min_order", _min_order, 3);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    _poly_num1D = 2 * _dev_order;

    nh.param("pillar1_x", pillar1(0), 0.0);
    nh.param("pillar1_y", pillar1(1), 0.0);
    nh.param("pillar1_z", pillar1(2), 0.0);

    nh.param("pillar2_x", pillar2(0), 0.0);
    nh.param("pillar2_y", pillar2(1), 0.0);
    nh.param("pillar2_z", pillar2(2), 0.0);

    nh.param("pillar3_x", pillar3(0), 0.0);
    nh.param("pillar3_y", pillar3(1), 0.0);
    nh.param("pillar3_z", pillar3(2), 0.0);

    nh.param("pillar4_x", pillar4(0), 0.0);
    nh.param("pillar4_y", pillar4(1), 0.0);
    nh.param("pillar4_z", pillar4(2), 0.0);

    nh.param("terminal_apriltag_x", terminal_apriltag(0), 0.0);
    nh.param("terminal_apriltag_y", terminal_apriltag(1), 0.0);
    nh.param("terminal_apriltag_z", terminal_apriltag(2), 0.0);

    nh.param("time_forward", time_forward_, -1.0);

    circle_dec.reset(new CIRCLE());

    //调用PlanningVisualization构造函数进行初始化
    trajVisual_.reset(new PlanningVisualization(nh));
    //柱子可视化
    trajVisual_->displayPillars(vector<Eigen::Vector3d>{pillar1, pillar2, pillar3, pillar4});

    odom_sub            = nh.subscribe("odom", 100, &CONSOLE::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    depth_sub           = nh.subscribe("depth", 100, &CONSOLE::DepthCallback, this, ros::TransportHints().tcpNoDelay());
    ego_planner_cmd_sub = nh.subscribe("planner_cmd", 100, &CONSOLE::CmdForwardCallback, this, ros::TransportHints().tcpNoDelay());
    apriltag_sub        = nh.subscribe("/tag_detections", 100, &CONSOLE::ApriltagCallback, this, ros::TransportHints().tcpNoDelay());
    color_sub           = nh.subscribe("image", 100, &CONSOLE::ColorImageCallback, this, ros::TransportHints().tcpNoDelay());

    trajPath_pub        = nh.advertise<nav_msgs::Path>("trajectory", 10);
    controlCmd_pub      = nh.advertise<quadrotor_msgs::PositionCommand>("/console_position_cmd", 50);
    land_pub            = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    vel_cmd_pub         = nh.advertise<geometry_msgs::TwistStamped>("/linear_cmd", 10);
    ego_planner_wps_pub = nh.advertise<nav_msgs::Path>("/ego_planner_wps", 1);
    console_state_pub   = nh.advertise<console::ConsoleState>("/console/state", 1);
    
    ControlCmdCallback_Timer = nh.createTimer(ros::Duration(0.01), &CONSOLE::ControlCmdCallback, this);
    Transformer_Timer        = nh.createTimer(ros::Duration(0.01), &CONSOLE::TransformerCallback, this);
    FSM_Timer                = nh.createTimer(ros::Duration(0.01), &CONSOLE::ControlFSM, this);
}


void CONSOLE::ColorImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (state == STATE::PASSING_CIRCLES || 1) {
        circle_pos = circle_dec->DetectionCallback(msg);
        if (circle_pos.first) {
            cout << "u: " << "(" << circle_pos.second.x << "," << circle_pos.second.y << ")" << endl;;
        }
    }
}

vector<geometry_msgs::Point> CONSOLE::Ego_planner_firstTaskWps_genaration() {
    vector<geometry_msgs::Point> firstTaskWps;
    geometry_msgs::Point tmp;
    tmp.x = pillar1(0); tmp.y = pillar1(1) + 1.4; tmp.z = 1;
    firstTaskWps.push_back(tmp);

    tmp.x = (pillar1(0) + pillar2(0)) / 2; tmp.y = (pillar1(1) + pillar2(1)) / 2; tmp.z = 1;
    firstTaskWps.push_back(tmp);

    tmp.x = pillar2(0); tmp.y = pillar2(1) - 1.8; tmp.z = 1;
    firstTaskWps.push_back(tmp);

    tmp.x = (pillar2(0) + pillar3(0)) / 2; tmp.y = (pillar2(1) + pillar3(1)) / 2; tmp.z = 1;
    firstTaskWps.push_back(tmp);

    tmp.x = pillar3(0); tmp.y = pillar3(1) + 1.4; tmp.z = 1;
    firstTaskWps.push_back(tmp);

    tmp.x = (pillar3(0) + pillar4(0)) / 2; tmp.y = (pillar3(1) + pillar4(1)) / 2; tmp.z = 1;
    firstTaskWps.push_back(tmp);

    tmp.x = pillar4(0); tmp.y = pillar4(1) - 1.8; tmp.z = 1;
    firstTaskWps.push_back(tmp);

    tmp.x = pillar4(0) + 1; tmp.y = pillar4(1) + 1.0; tmp.z = 1;
    pillars_terminal_position = {pillar4(0) + 1, pillar4(1) + 1.0, 1};
    firstTaskWps.push_back(tmp);   

    return firstTaskWps;
}

void CONSOLE::ApriltagCallback(const apriltag_ros::AprilTagDetectionArrayPtr& msg) {
    if (msg->detections.size() == 1) {
        apriltag_detected = true;
        Eigen::Vector3d local_apriltag_pos;
        local_apriltag_pos.x() = msg->detections[0].pose.pose.pose.position.y;
        local_apriltag_pos.y() = msg->detections[0].pose.pose.pose.position.x;
        local_apriltag_pos.z() = msg->detections[0].pose.pose.pose.position.z;

        apriltag_pos = odom_pos - odom_q * local_apriltag_pos;
        apriltag_pos(0) -= 0.2;
        apriltag_pos(1) -= 0.2;

        static int cnt = 0;
        if (cnt % 500 == 0) {
            cout << "apriltag: " << "(" << apriltag_pos(0) << "," << apriltag_pos(1) << "," << apriltag_pos(2) << ")" << endl;
        }
        cnt++;
    }
    
}
void CONSOLE::DepthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    have_depth = true;
    cv_bridge::CvImagePtr tmp = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
    depth_image = tmp->image;
}


void CONSOLE::TransformerCallback(const ros::TimerEvent &e){
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "camera_odom_frame";
    transform.transform.translation.x = 0.0;  // 单位平移x坐标
    transform.transform.translation.y = 0.0;  // 单位平移y坐标
    transform.transform.translation.z = 0.0;  // 单位平移z坐标
    transform.transform.rotation.x = 0.0;  // 单位四元数x分量
    transform.transform.rotation.y = 0.0;  // 单位四元数y分量
    transform.transform.rotation.z = 0.0;  // 单位四元数z分量
    transform.transform.rotation.w = 1.0;  // 单位四元数w分量
    broadcaster.sendTransform(transform);    
}

void CONSOLE::OdomCallback(const nav_msgs::OdometryConstPtr msg){
    odom_pos(0) = msg->pose.pose.position.x;
    odom_pos(1) = msg->pose.pose.position.y;
    odom_pos(2) = msg->pose.pose.position.z;

    odom_q.x() = msg->pose.pose.orientation.x;
    odom_q.y() = msg->pose.pose.orientation.y;
    odom_q.z() = msg->pose.pose.orientation.z;
    odom_q.w() = msg->pose.pose.orientation.w;
    has_odom = true;
}


void CONSOLE::waypointsVisual(const Eigen::MatrixXd& path){
    vector<Eigen::Vector3d> wp_list;
    for (int k = 0; k < path.rows(); k++)
    {
        Vector3d pt(path(k, 0), path(k, 1), path(k, 2));
        wp_list.push_back(pt);
        ROS_INFO("waypoint%d: (%f, %f, %f)", k+1, pt(0), pt(1), pt(2));
    }
    //路标可视化
    trajVisual_->displayWaypoints(wp_list);
}






