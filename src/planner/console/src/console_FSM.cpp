#include "console/console.hpp"

void CONSOLE::ControlFSM(const ros::TimerEvent &e){
  int current_state = state;

  if (!has_odom && odom_pos(2) < 1) {
    state = WAIT_TAKE_OFF_FINISHED;

  }else if (has_odom && abs(odom_pos(2) - 1) < 0.1 && !is_complete_Start_Traj) {

    /****使用minimumsnap******/
    Eigen::MatrixXd waypoints(2, 3);
    Eigen::Vector3d statr_pos(odom_pos(0), odom_pos(1), 1);
    waypoints.row(0) = statr_pos;
    waypoints.row(1) << pillar1(0) - 1, pillar1(1) + 1.5, 1;      
    waypointsVisual(waypoints);
    Traj_start = trajGeneration(waypoints);
    is_complete_Start_Traj = true;

    /********使用ego-planner*************/

    state = FLY_TO_STRAT;

  }else if (state == FLY_TO_STRAT && has_odom && start_trajectory_finished && !pillar_detected) {

    posiCmd.header.stamp = ros::Time::now();

    posiCmd.position.x = pillar1(0) - 1;
    posiCmd.position.y = pillar1(1) + 1.5;
    posiCmd.position.z = 1;

    posiCmd.velocity.x = 0;
    posiCmd.velocity.y = 0;
    posiCmd.velocity.z = 0;

    /*偏航调整为-90度，保证其正对圆柱进行搜索*/
    static bool rotate_finished = false;
    if (!rotate_finished) {
      ros::Rate loop(100);
      double diff_yaw = (-3.14 / 2 - last_yaw_) / 100;
      for (int i = 1; i <= 100; i++) {
        posiCmd.yaw = last_yaw_ + diff_yaw;
        posiCmd.yaw_dot = 0;
        controlCmd_pub.publish(posiCmd);
        last_yaw_ = posiCmd.yaw;
        loop.sleep();
      }
      rotate_finished = true;
    }
    /*************************************/
    

    //等待1s,确保转向完成
    ros::Rate jj(100);
    for (int i = 0; i < 100; i++) jj.sleep();

    state = SEARCHING_PILLAR;

  }else if (state == SEARCHING_PILLAR && has_odom && state != ROTATE_PILLARS && pillar_detected) {  
    //等待1s
    if (state != ROTATE_PILLARS) {
      ros::Rate loop1(100);
      for (int i = 0; i < 100; i++) loop1.sleep();
    }

    /*ego_planner方案*/
    // nav_msgs::Path first_task_path;
    // first_task_path.header.stamp    = ros::Time::now();
    // first_task_path.header.frame_id = "world";
    // vector<geometry_msgs::Point> firstTaskWps = Ego_planner_firstTaskWps_genaration();
    // for (size_t i = 0; i < firstTaskWps.size(); i++) {
    //   geometry_msgs::PoseStamped tmp_Pos;
    //   tmp_Pos.pose.position = firstTaskWps[i];
    //   first_task_path.poses.push_back(tmp_Pos);
    // }
    // ego_planner_wps_pub.publish(first_task_path);

    state = ROTATE_PILLARS;
     
  }else if (state == ROTATE_PILLARS && rotate_pillars_finished){
    ros::Rate loop2(100);
    for (int i = 0; i < 100; i++) loop2.sleep();

    Eigen::MatrixXd fly2apriltag(2, 3);
    Eigen::Vector3d fly2apriltag_start(odom_pos(0), odom_pos(1), 1);
    fly2apriltag.row(0) = fly2apriltag_start;
    fly2apriltag.row(1) << terminal_apriltag(0), terminal_apriltag(1), 1;      
    waypointsVisual(fly2apriltag);
    fly2apriltag_traj = trajGeneration(fly2apriltag);


    state = SEARCHING_APRILTAG;
    
  }
  else if (state == SEARCHING_APRILTAG && fly2land_finished) {
    ros::Rate loop3(100);
    for (int i = 0; i < 100; i++) loop3.sleep();
    state = LANDING;
  }

  const char* logMessage;
  // 检查状态转换，并使用 `ROS_WARN` 只在状态改变时打印。
  if (current_state != state) {
    switch (state) {
      case INIT:
        logMessage = "STATE: INIT";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case WAIT_TAKE_OFF_FINISHED:
        logMessage = "STATE: WAIT_TAKE_OFF_FINISHED";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case FLY_TO_STRAT:
        logMessage = "STATE: FLY_TO_STRAT";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case SEARCHING_PILLAR:
        logMessage = "STATE: SEARCHING_PILLAR";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case ROTATE_PILLARS:
        logMessage = "STATE: ROTATE_PILLARS";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case SEARCHING_APRILTAG:
        logMessage = "STATE: SEARCHING_APRILTAG";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case LANDING:
        logMessage = "STATE: LANDING";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      default:
        break;
    }
  }

  console::ConsoleState msg;
  msg.state = state;
  console_state_pub.publish(msg);
}


void CONSOLE::ControlCmdCallback(const ros::TimerEvent &e)
{
  if (state == STATE::WAIT_TAKE_OFF_FINISHED) {
    return;
  }
  if (state == STATE::FLY_TO_STRAT) {

    //使用minimunmsnap
    static ros::Time fisrt_tra_start_time = ros::Time::now();
    static bool FLY_TO_STRAT_INIT = true;
    //飞向起点的过程不进行转向
    ControlParse(Traj_start, fisrt_tra_start_time, FLY_TO_STRAT_INIT, start_trajectory_finished, 0);
    FLY_TO_STRAT_INIT = false;
  }

  if (state == STATE::SEARCHING_PILLAR) {
    static double range_x = 0.1;
    static double cur_y = odom_pos(1);

    posiCmd.header.stamp = ros::Time::now();
    posiCmd.position.x = odom_pos(0) + range_x;
    posiCmd.position.y = cur_y;
    posiCmd.position.z = 1;

    posiCmd.velocity.x = 0.1;
    posiCmd.velocity.y = 0;
    posiCmd.velocity.z = 0;

    posiCmd.yaw = last_yaw_;
    posiCmd.yaw_dot = 0;
    last_yaw_ = posiCmd.yaw;
    controlCmd_pub.publish(posiCmd);

    auto result = findPillar();
    if (result.first && !pillar_detected) {
      ROS_WARN("pillar find!!!");
      ros::Rate kep(100);
      for (int j = 0; j < 50; j++) kep.sleep();

      /***绕柱路标点***/
      Eigen::MatrixXd rotate_pillars_wps(4, 3);

      double dis2pillar = findPillar().second;

      cout << "dis2pillar: " << dis2pillar << endl;

      Eigen::Vector3d pillar_position;

      dis2pillar -= 0.1;

      pillar_position.x() = odom_pos(0);
      pillar_position.y() = odom_pos(1) - dis2pillar;
      pillar_position.z() = odom_pos(2);


      rotate_pillars_wps.row(0) << pillar_position(0) + 0.3, pillar_position(1) + dis2pillar, 1;
      rotate_pillars_wps.row(1) << pillar_position(0) + 1 + 0.2, pillar_position(1) - dis2pillar, 1;
      rotate_pillars_wps.row(2) << pillar_position(0) + 2, pillar_position(1) + dis2pillar, 1;
      rotate_pillars_wps.row(3) << pillar_position(0) + 3, pillar_position(1) - dis2pillar, 1;
      // rotate_pillars_wps.row(4) << pillar_position(0) + 4, pillar_position(1), 1;
      /***************/
      waypointsVisual(rotate_pillars_wps);
      rotate_pillars_traj = trajGeneration(rotate_pillars_wps);
      pillar_detected = true;
    }

 
  }

  if (state == STATE::ROTATE_PILLARS) {

    /*minimumsnap方案*/
    static ros::Time rotate_pillars_start_time = ros::Time::now();
    static bool rotate_pillars_init = true;
    ControlParse(rotate_pillars_traj, rotate_pillars_start_time, rotate_pillars_init, rotate_pillars_finished, 1);
    rotate_pillars_init = false;
  }

  if (state == STATE::SEARCHING_APRILTAG) {


    static bool plan_finished = false;

    if (!plan_finished) {
      static ros::Time fly2apriltag_start_time = ros::Time::now();
      static bool fly2apriltag_init = true;
      ControlParse(fly2apriltag_traj, fly2apriltag_start_time, fly2apriltag_init, fly2apriltag_finished, 1);
      fly2apriltag_init = false;
    }

    if (apriltag_detected && fly2apriltag_finished && !plan_finished) {
      Eigen::MatrixXd fly2land(2, 3);
      Eigen::Vector3d fly2land_start(odom_pos(0), odom_pos(1), 1);
      fly2land.row(0) = fly2land_start;
      fly2land.row(1) << apriltag_pos(0), apriltag_pos(1), 1;      
      waypointsVisual(fly2land);
      fly2land_traj = trajGeneration(fly2land);
      plan_finished = true;
    }
    if (plan_finished) {
      static ros::Time fly2land_start_time = ros::Time::now();
      static bool fly2land_init = true;
      ControlParse(fly2land_traj, fly2land_start_time, fly2land_init, fly2land_finished, 0);
      fly2land_init = false;
    }

    if (fly2apriltag_finished && !apriltag_detected) state = LANDING;


    // static bool has_pub = false;
    // ros::Rate waiting(100);
    // //ego_planner方案
    // for (int i = 0; i < 300 && !has_pub; i++) {
    //   if (apriltag_detected) {
    //     nav_msgs::Path world_apriltag_pos;
    //     world_apriltag_pos.header.stamp    = ros::Time::now();
    //     world_apriltag_pos.header.frame_id = "world";
    //     geometry_msgs::PoseStamped tmp;
    //     tmp.pose.position.x = apriltag_pos(0);
    //     tmp.pose.position.y = apriltag_pos(1);
    //     tmp.pose.position.z = apriltag_pos(2) + 0.5;
    //     world_apriltag_pos.poses.push_back(tmp);
    //     ego_planner_wps_pub.publish(world_apriltag_pos);
    //     has_pub = true;
    //     break;
    //   }
    //   waiting.sleep();
    // }

    //如果五秒后还没有搜到就直接降落
    //if (!has_pub) state = LANDING;

  }

  if (state == STATE::LANDING) {
    if (!is_land){
      ros::Rate wait_cmd(100);
      for (int i = 0; i < 50; i++) wait_cmd.sleep();
      quadrotor_msgs::TakeoffLand cmd;
      cmd.takeoff_land_cmd = 2;
      land_pub.publish(cmd);
      is_land = true;
    }
  }
}

std::pair<bool, double> CONSOLE::findPillar() {
  double dis_sum = 0;
  int cnt = 0;
	for (int i = 240 - 15; i < 240 + 15 && have_depth;  ++i)  {
	  ushort* p = depth_image.ptr<ushort>(i);

    if(p[320] != 0 && p[320] != 65535) {
      dis_sum += double(p[320]);
      cnt++;
      }   
  }

  double ave_sum = 0;
  if (cnt != 0) {
    ave_sum = (dis_sum / cnt) / 1000; 
  }
  static double pre_dis = ave_sum;

  if (ave_sum == 0.0) {
    ave_sum = pre_dis;
  }

  bool res = (pre_dis != 0 && ave_sum != 0 && ave_sum < 2.5 && (pre_dis - ave_sum) > 0.5) ? true : false;
  
  static int hh = 0;
  if (hh % 50 == 0 ) {
    pre_dis = ave_sum;
    cout << ave_sum << endl;
  }
  hh++;  
  
  std::pair<bool, double> res_dis(res, ave_sum);

  return res_dis;
}

/*该函数解析ego_planner发布的控制指令，console做一个转发*/
void CONSOLE::CmdForwardCallback(const quadrotor_msgs::PositionCommandPtr msg) {
  //此处为0代表先不转发
  if (0) {
    posiCmd = *msg;
    controlCmd_pub.publish(posiCmd);
  }
}

void CONSOLE::ControlParse(MiniSnapTraj& trajectory, ros::Time start_time, bool init, bool& finished, bool isAdjustYaw) {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time).toSec();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0); 
    static ros::Time time_last = ros::Time::now();

    if (init) {
      time_last = ros::Time::now();
    } 
    if (t_cur < trajectory.time_duration && t_cur >= 0.0){
      auto info = getTrajInfo(trajectory, t_cur);
      pos = info.first;
      vel = info.second;
      yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last, trajectory);

    }else if (t_cur >= trajectory.time_duration){
      auto info = getTrajInfo(trajectory, trajectory.time_duration);
      pos = info.first;
      vel.setZero();
      yaw_yawdot.first = last_yaw_;
      yaw_yawdot.second = 0;
      finished = true;

    }else{
      cout << "[trajectory server]: invalid time." << endl;
    }

    time_last = time_now;

    posiCmd.header.stamp = time_now;
    posiCmd.header.frame_id = "world";
    posiCmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    posiCmd.trajectory_id = 1;

    posiCmd.position.x = pos(0);
    posiCmd.position.y = pos(1);
    posiCmd.position.z = pos(2);

    posiCmd.velocity.x = vel(0);
    posiCmd.velocity.y = vel(1);
    posiCmd.velocity.z = vel(2);

    
    if (isAdjustYaw) {
      posiCmd.yaw = yaw_yawdot.first;
    }else {
      posiCmd.yaw = last_yaw_;
    }
    
    posiCmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = posiCmd.yaw;
    controlCmd_pub.publish(posiCmd);
}


std::pair<double, double> CONSOLE::calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last, MiniSnapTraj& trajectory){
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= trajectory.time_duration ? getTrajInfo(trajectory, t_cur + time_forward_).first - pos : getTrajInfo(trajectory, trajectory.time_duration).first - pos;

  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;    
}







