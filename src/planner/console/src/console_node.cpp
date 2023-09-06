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
    nh.param("time_forward", time_forward_, -1.0);

    //相机内参
    nh.param("fx_", fx, 606.7078247070312);
    nh.param("fy_", fy, 606.3765869140625);
    nh.param("cx_", cx, 321.51348876953125);
    nh.param("cy_", cy, 258.6695556640625);

    camera_inner_matrix <<  fx , 0 , cx,
                            0 ,  fy, cy,
                            0 ,  0 ,  1; 

    nh.param("fly_altitude", fly_altitude, 1.5);

    //第一关柱子坐标
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


    //第二关隧道起终点坐标
    nh.param("tunnel_start_x", tunnel_start(0), 0.0);
    nh.param("tunnel_start_y", tunnel_start(1), 0.0);
    nh.param("tunnel_start_z", tunnel_start(2), 0.0);

    // nh.param("tunnel_end_x", tunnel_end(0), 0.0);
    // nh.param("tunnel_end_y", tunnel_end(1), 0.0);

    tunnel_end(0) = tunnel_start(0) + 3.567;
    tunnel_end(1) = tunnel_start(1) - 1.81;
    tunnel_end(2) = fly_altitude;
    

    //第三关迷宫路标点
    nh.param("maze_wp1_x", maze_wp1(0), 0.0);
    nh.param("maze_wp1_y", maze_wp1(1), 0.0);
    nh.param("maze_wp1_z", maze_wp1(2), 0.0);

    nh.param("maze_wp2_x", maze_wp2(0), 0.0);
    nh.param("maze_wp2_y", maze_wp2(1), 0.0);
    nh.param("maze_wp2_z", maze_wp2(2), 0.0);

    nh.param("maze_wp3_x", maze_wp3(0), 0.0);
    nh.param("maze_wp3_y", maze_wp3(1), 0.0);
    nh.param("maze_wp3_z", maze_wp3(2), 0.0);

    nh.param("maze_wp4_x", maze_wp4(0), 0.0);
    nh.param("maze_wp4_y", maze_wp4(1), 0.0);
    nh.param("maze_wp4_z", maze_wp4(2), 0.0);


    nh.param("planning_circle_altitude", planning_circle_altitude, 1.7);
    nh.param("searching_apriltag_altitude", searching_apriltag_altitude, 1.7);
    nh.param("circle_D", circle_D, 1.0);
    nh.param("planning_circle_radius", planning_circle_radius, 150);

    //二维码终点预设坐标
    nh.param("terminal_apriltag_x", terminal_apriltag(0), 0.0);
    nh.param("terminal_apriltag_y", terminal_apriltag(1), 0.0);
    nh.param("terminal_apriltag_z", terminal_apriltag(2), 0.0);

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
    vins_sub            = nh.subscribe("/vins_fusion/imu_propagate", 100, &CONSOLE::VinsOdomCallback, this, ros::TransportHints().tcpNoDelay());
   
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

void CONSOLE::UV2Camera(double dis) {
    cam_pos = dis * camera_inner_matrix.inverse() * point2D_h;
    cam2body_matrix <<  0 , 0 , 1,
                        -1, 0 , 0,
                        0, -1 , 0;

    Eigen::Matrix<float,3,1> move_ori;
    move_ori << 0.06, 0 , 0.15;  

    cam_pos = cam2body_matrix * cam_pos + move_ori;

    Eigen::Vector3d body_pos(cam_pos(0, 0) + 2 , cam_pos(1, 0), cam_pos(2, 0) ); 


    circle_world_pos = odom_q * body_pos + odom_pos;

    static int huhuhu = 0;
    huhuhu++;
    if (huhuhu % 1 == 0) {
        cout << "distance: " << dis << endl;
        cout << "camera's target position: " << "(" << circle_world_pos.x() << "," <<  circle_world_pos.y() << "," << circle_world_pos.z() << ")" << endl;
       // cout << "camera's target position: " << "(" << cam_pos(0, 0) << "," << cam_pos(1, 0) << "," << cam_pos(2, 0) << ")" << endl;
    }

}



double CONSOLE::LimitOutput(double val, double max_) {
    if (val > -max_ && val < max_) {
        return val;
    }else if (val >= max_) {
        return max_;
    }else {
        return -max_;
    }
}

void onTrackbarChange(int value, void* userData) {
    // 在这里可以处理滑动条值的变化，例如对图像进行处理
}



void CONSOLE::ColorImageCallback(const sensor_msgs::Image::ConstPtr& msg) {

    // static bool flls = true;
    // if (flls) {
    //     namedWindow("Image Window", WINDOW_NORMAL);
    //     int sliderValue = 0;  // 初始值
    //     createTrackbar("Slider", "Image Window", &sliderValue, 1000, onTrackbarChange);
    //     flls = false;
    // }


    if (state == STATE::PASSING_CIRCLES && !toward_circle_finished) {
    // if (1) {
        static int circle_miss_cnt = 0;
        
        circle_pos = circle_dec->DetectionCallback(msg);
        if (circle_pos.first ) {
            circle_miss_cnt = 0;
            have_circle = true;

            have_circle_cnt++;
            // cout << "pos: " << "(" << circle_pos.second.x << "," << circle_pos.second.y << ")" << endl;
            cout << "r: " << circle_pos.second.z << endl;

            // // 获取滑条的值
            // int currentValue = getTrackbarPos("Slider", "Image Window");

            // cout << "curccc: " << currentValue << endl; 
            circle_dis = (314.4444444444444 * circle_D) / circle_pos.second.z;

            point2D_h << circle_pos.second.x, circle_pos.second.y, 1;
            UV2Camera(circle_dis);


            // cout << "the real depth: " << circle_dis << endl;

        }else {
            static bool start_miss = false;

            //加这个标志位是为了保证只有检测到30帧圆之后才会进入丢失的判断
            if (have_circle_cnt >= 30) start_miss = true;

            //加这个标志位是为了保证只有在检测到30帧圆之后持续漏检才认为圆已经很近了
            if (start_miss){
                circle_miss_cnt++;
            } 

            if (circle_miss_cnt == 60)
            {
                passed_circle_flag = true;
                ROS_WARN("get done");
            }

            have_circle = false;
        }
    }
}


//机体系速度控制
void CONSOLE::BodyMoveControlParse(Eigen::Vector3d& ori, double height) {

    static Eigen::Vector3d last_ori = ori;
    static Eigen::Vector3d odom_con = odom_pos;
    static double x_con = odom_con(0);
    static double y_con = odom_con(1);

    // vel_state = MOVESIDE;


    if (last_ori != ori) {
        ROS_WARN("SWITCH!");
        odom_con = odom_pos;
        x_con = odom_con(0);
        y_con = odom_con(1);
    }


    //机体转世界坐标系
    Eigen::Vector3d world = odom_q * ori;

    //world(1) = -world(1);
    
    x_con += world(0);
    y_con += world(1);

    static int cnt = 0;
    cnt++;
    if (cnt % 10 == 0) {
        cout << "world " << x_con -  odom_pos(0)<< "   ";
        cout << y_con - odom_pos(1)<< endl;
    }

    posiCmd.header.stamp = ros::Time::now();
    posiCmd.position.x = x_con;
    posiCmd.position.y = y_con;

    posiCmd.position.z = height;

    posiCmd.velocity.x = world(0) * 100;
    posiCmd.velocity.y = world(1) * 100;
    posiCmd.velocity.z = 0;
    posiCmd.yaw = last_yaw_;
    posiCmd.yaw_dot = 0;
    last_yaw_ = posiCmd.yaw;
    controlCmd_pub.publish(posiCmd);

    last_ori = ori;
}


//圆调整函数
void CONSOLE::BodyForwardControlParse(Eigen::Vector3d& ori) {

    Eigen::Vector3d world = odom_q * ori;

    static int cnggt = 0;
    cnggt++;
    if (cnggt % 10 == 0) {
        cout << "cirle_adjust_world " << world(0) << "   ";
        cout << world(1) << endl;
    }


    posiCmd.header.stamp = ros::Time::now();
    posiCmd.position.x = odom_pos(0) + world(0);
    posiCmd.position.y = odom_pos(1) + world(1);
    posiCmd.position.z = planning_circle_altitude;

    posiCmd.velocity.x = world(0);
    posiCmd.velocity.y = world(1);
    posiCmd.velocity.z = 0;

    posiCmd.yaw = last_yaw_;
    posiCmd.yaw_dot = 0;
    last_yaw_ = posiCmd.yaw;
    controlCmd_pub.publish(posiCmd);
}




vector<geometry_msgs::Point> CONSOLE::Ego_planner_mazeWps_genaration() {
    vector<geometry_msgs::Point> mazeWps;
    geometry_msgs::Point tmp;

    tmp.x = maze_wp1(0); tmp.y = maze_wp1(1); tmp.z = maze_wp1(2);
    mazeWps.push_back(tmp);

    tmp.x = maze_wp2(0); tmp.y = maze_wp2(1); tmp.z = maze_wp2(2);
    mazeWps.push_back(tmp);

    tmp.x = maze_wp3(0); tmp.y = maze_wp3(1); tmp.z = maze_wp3(2);
    mazeWps.push_back(tmp);  

    tmp.x = maze_wp4(0); tmp.y = maze_wp4(1); tmp.z = maze_wp4(2);
    mazeWps.push_back(tmp);        

    return mazeWps;
}

void CONSOLE::ApriltagCallback(const apriltag_ros::AprilTagDetectionArrayPtr& msg) {
    if (msg->detections.size() == 1) {
        apriltag_detected = true;
        Eigen::Vector3d local_apriltag_pos;
        local_apriltag_pos.x() = msg->detections[0].pose.pose.pose.position.y;
        local_apriltag_pos.y() = msg->detections[0].pose.pose.pose.position.x;
        local_apriltag_pos.z() = msg->detections[0].pose.pose.pose.position.z;

        //apriltag_pos = odom_pos - odom_q * local_apriltag_pos;
        apriltag_pos = odom_pos - local_apriltag_pos;
        // apriltag_pos(0) -= 0.2;
        // apriltag_pos(1) -= 0.2;

        static int cnt = 0;
        if (cnt % 5 == 0) {
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

    odom_vel(0) = msg->twist.twist.linear.x;
    odom_vel(1) = msg->twist.twist.linear.y;
    odom_vel(2) = msg->twist.twist.linear.z;

    odom_q.x() = msg->pose.pose.orientation.x;
    odom_q.y() = msg->pose.pose.orientation.y;
    odom_q.z() = msg->pose.pose.orientation.z;
    odom_q.w() = msg->pose.pose.orientation.w;

    // double yaw = atan2(2 * (odom_q.x()*odom_q.y() + odom_q.w()*odom_q.z()), odom_q.w()*odom_q.w() + odom_q.x()*odom_q.x() - odom_q.y()*odom_q.y() - odom_q.z()*odom_q.z());
    // cout << "yaw:" << yaw << endl;
    has_odom = true;
}

void CONSOLE::VinsOdomCallback(const nav_msgs::OdometryConstPtr msg) {
    vins_altitude = msg->pose.pose.position.z;
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






