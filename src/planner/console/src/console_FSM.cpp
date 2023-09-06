#include "console/console.hpp"

void CONSOLE::ControlFSM(const ros::TimerEvent &e){
  int current_state = state;

  // state = PASSING_CIRCLES;

  if (!has_odom && odom_pos(2) < 1) {
    state = WAIT_TAKE_OFF_FINISHED;

  }else if (has_odom && abs(odom_pos(2) - 1.5) < 0.1 && !is_complete_Start_Traj) {

    ros::Rate waitjj(100);
    for (int i = 0; i < 100; i++) {
      waitjj.sleep();
    }
    /****使用minimumsnap******/
    Eigen::MatrixXd waypoints(2, 3);
    Eigen::Vector3d statr_pos(odom_pos(0), odom_pos(1), fly_altitude);
    //Eigen::Vector3d statr_pos(0, 0, fly_altitude);
    waypoints.row(0) = statr_pos;
    waypoints.row(1) << pillar1(0) - 1.0, pillar1(1) + 1.2, fly_altitude;      
    waypointsVisual(waypoints);
    Traj_start = trajGeneration(waypoints);
    is_complete_Start_Traj = true;

    state = FLY_TO_STRAT;
    // state = PASSING_CIRCLES;


  }else if (state == FLY_TO_STRAT && has_odom && start_trajectory_finished && !pillar_detected) {

    posiCmd.header.stamp = ros::Time::now();

    posiCmd.position.x = pillar1(0) - 1.0;
    posiCmd.position.y = pillar1(1) + 1.2;
    posiCmd.position.z = fly_altitude;

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

    //等待0.5s,确保转向完成
    ros::Rate jj(100);
    for (int i = 0; i < 100; i++) jj.sleep();

    state = SEARCHING_PILLAR;

  }else if (state == SEARCHING_PILLAR && has_odom && state != ROTATE_PILLARS && pillar_detected) {  
    //等待1s
    if (state != ROTATE_PILLARS) {
      ros::Rate loop1(100);
      for (int i = 0; i < 50; i++) loop1.sleep();
    }

    state = ROTATE_PILLARS;
     
  }else if (state == ROTATE_PILLARS && rotate_pillars_finished){

    //规划飞向隧道起点的轨迹
    Eigen::MatrixXd fly2tunnel_start(2, 3);
    Eigen::Vector3d fly2tunnel_start_start(odom_pos(0), odom_pos(1), fly_altitude);
    fly2tunnel_start.row(0) = fly2tunnel_start_start;
    //fly2tunnel_start.row(1) << tunnel_start(0), tunnel_start(1), fly_altitude; 
    fly2tunnel_start.row(1) << maze_wp4(0), maze_wp4(1), fly_altitude;      
    waypointsVisual(fly2tunnel_start);
    fly2tunnel_start_traj = trajGeneration(fly2tunnel_start);

   // state = FLY_TO_TUNNEL_STRAT;
    state = SEARCHING_MAZE;

  }else if (state == FLYING_VIA_TUNNEL && tunnel_finished){

    /*ego_planner方案,跟踪迷宫关卡固定的路标点*/
    // nav_msgs::Path maze_path;
    // maze_path.header.stamp    = ros::Time::now();
    // maze_path.header.frame_id = "world";
    // vector<geometry_msgs::Point> tunnelWps = Ego_planner_mazeWps_genaration();
    // for (size_t i = 0; i < tunnelWps.size(); i++) {
    //   geometry_msgs::PoseStamped tmp_Pos;
    //   tmp_Pos.pose.position = tunnelWps[i];
    //   maze_path.poses.push_back(tmp_Pos);
    // }
    // ego_planner_wps_pub.publish(maze_path);


    //第一条轨迹
    Eigen::MatrixXd first_search_maze(2, 3);
    first_search_maze.row(0) << tunnel_end(0), tunnel_end(1), planning_circle_altitude; 
    first_search_maze.row(1) << maze_wp1(0), maze_wp1(1), planning_circle_altitude;      
    waypointsVisual(first_search_maze);
    first_search_maze_traj = trajGeneration(first_search_maze);

    //第二条轨迹
    Eigen::MatrixXd second_search_maze(2, 3);
    second_search_maze.row(0) << maze_wp1(0), maze_wp1(1), planning_circle_altitude; 
    second_search_maze.row(1) << maze_wp4(0), maze_wp4(1), planning_circle_altitude;      
    waypointsVisual(second_search_maze);
    second_search_maze_traj = trajGeneration(second_search_maze);


    state = SEARCHING_MAZE;

  }else if (state == SEARCHING_MAZE && fly2tunnel_finished){
    circle_adjust_yaw = true;


    //穿圈之前先把偏航调整为正对着圆
    posiCmd.header.stamp = ros::Time::now();
    posiCmd.position.x = maze_wp4(0);
    posiCmd.position.y = maze_wp4(1);
    posiCmd.position.z = planning_circle_altitude;

    posiCmd.velocity.x = 0;
    posiCmd.velocity.y = 0;
    posiCmd.velocity.z = 0;

    double ter_yaw;
    if (last_yaw_ < 0) ter_yaw = -3.14;
    else ter_yaw = 3.14;

    static double diff = (ter_yaw - last_yaw_) / 100;
    ros::Rate ju(100);
    for (int i = 1; i <= 100; i++) {
      posiCmd.yaw = last_yaw_ + diff;
      posiCmd.yaw_dot = 0;
      last_yaw_ = posiCmd.yaw;
      controlCmd_pub.publish(posiCmd);
      ju.sleep(); 
    }

    ros::Rate hhhhhhh(100);
    for (int i = 1; i < 100; i++) {
      hhhhhhh.sleep();
    }

    state = PASSING_CIRCLES;

  }else if (state == PASSING_CIRCLES && toward_circle_finished){
    ros::Rate loop2(100);
    for (int i = 0; i < 50; i++) loop2.sleep();

    //规划一个飞向圆终点右侧的轨迹
    Eigen::MatrixXd cricle2apriltag_wps(2, 3);
    cricle2apriltag_wps.row(0) << cricle2apriltag(0), cricle2apriltag(1) - 5, searching_apriltag_altitude;
    cricle2apriltag_wps.row(1) << cricle2apriltag(0), cricle2apriltag(1), searching_apriltag_altitude;      
    waypointsVisual(cricle2apriltag_wps);
    cricle2apriltag_traj = trajGeneration(cricle2apriltag_wps);

    //规划飞向终点的轨迹
    Eigen::MatrixXd fly2apriltag(2, 3);
    fly2apriltag.row(0) << cricle2apriltag(0), cricle2apriltag(1), searching_apriltag_altitude;
    fly2apriltag.row(1) << terminal_apriltag(0), terminal_apriltag(1), searching_apriltag_altitude;      
    waypointsVisual(fly2apriltag);
    fly2apriltag_traj = trajGeneration(fly2apriltag);

    state = SEARCHING_APRILTAG;
    
  }
  // else if (state == SEARCHING_APRILTAG && fly2land_finished) {
  //   ros::Rate loop3(100);
  //   for (int i = 0; i < 100; i++) loop3.sleep();
  //   state = LANDING;
  // }

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
      case FLY_TO_TUNNEL_STRAT:
        logMessage = "STATE: FLY_TO_TUNNEL_STRAT";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case FLYING_VIA_TUNNEL:
        logMessage = "STATE: FLYING_VIA_TUNNEL";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case SEARCHING_MAZE:
        logMessage = "STATE: SEARCHING_MAZE";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case PASSING_CIRCLES:
        logMessage = "STATE: PASSING_CIRCLES";
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
    ControlParse(Traj_start, fisrt_tra_start_time, FLY_TO_STRAT_INIT, start_trajectory_finished, 1);
    FLY_TO_STRAT_INIT = false;
  }

  if (state == STATE::SEARCHING_PILLAR) {
    static double range_x = 0.1;
    static double cur_y = odom_pos(1);

    // posiCmd.header.stamp = ros::Time::now();
    // posiCmd.position.x = odom_pos(0) + range_x;
    // posiCmd.position.y = cur_y;
    // posiCmd.position.z = fly_altitude;

    // posiCmd.velocity.x = 0.2;
    // posiCmd.velocity.y = 0;
    // posiCmd.velocity.z = 0;

    // posiCmd.yaw = last_yaw_;
    // posiCmd.yaw_dot = 0;
    // last_yaw_ = posiCmd.yaw;
    // controlCmd_pub.publish(posiCmd);

    Eigen::Vector3d ser(0, 0.0028, 0);
    BodyMoveControlParse(ser, fly_altitude);

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

      rotate_pillars_wps.row(0) << pillar_position(0) + 0.5, pillar_position(1) + dis2pillar, fly_altitude;
      rotate_pillars_wps.row(1) << pillar_position(0) + 1.7 + 0.2, pillar_position(1) - dis2pillar, fly_altitude;
      rotate_pillars_wps.row(2) << pillar_position(0) + 3.4, pillar_position(1) + dis2pillar, fly_altitude;
      rotate_pillars_wps.row(3) << pillar_position(0) + 5.1, pillar_position(1) - dis2pillar, fly_altitude;
      // rotate_pillars_wps.row(4) << pillar_position(0) + 4, pillar_position(1), 1;
      /***************/
      waypointsVisual(rotate_pillars_wps);
      rotate_pillars_traj = trajGeneration(rotate_pillars_wps);
      pillar_detected = true;
    }
  }

  if (state == STATE::ROTATE_PILLARS) {

    /*跟踪给定的绕柱轨迹*/
    static ros::Time rotate_pillars_start_time = ros::Time::now();
    static bool rotate_pillars_init = true;
    ControlParse(rotate_pillars_traj, rotate_pillars_start_time, rotate_pillars_init, rotate_pillars_finished, 1);
    rotate_pillars_init = false;
  }


  if (state == STATE::FLY_TO_TUNNEL_STRAT) {
    /*先飞向隧道起点*/
    if (!fly2tunnel_finished) {
      static ros::Time fly2tunnel_start_time = ros::Time::now();
      static bool fly2tunnel_init = true;
      ControlParse(fly2tunnel_start_traj, fly2tunnel_start_time, fly2tunnel_init, fly2tunnel_finished, 1);
      fly2tunnel_init = false; 
    }

    /*绕完柱子后，调整飞机偏航角为正对隧道*/
    static bool yaw_adjust_finished = false;

    if(fly2tunnel_finished && !yaw_adjust_finished) {
      posiCmd.header.stamp = ros::Time::now();
      posiCmd.position.x = tunnel_start(0);
      posiCmd.position.y = tunnel_start(1);
      posiCmd.position.z = fly_altitude;

      posiCmd.velocity.x = 0;
      posiCmd.velocity.y = 0;
      posiCmd.velocity.z = 0;

      //在上升高度的过程中让无人机旋转到使其大致对准隧道入口
      static double diff = (-0.47 - last_yaw_) / 100;

      ros::Rate huj(100);
      for (int i = 1; i <= 100; i++) {
        posiCmd.yaw = last_yaw_ + diff;
        posiCmd.yaw_dot = 0;
        last_yaw_ = posiCmd.yaw;
        controlCmd_pub.publish(posiCmd);  
        huj.sleep(); 
      }
      
      //规划穿越隧道的轨迹
      Eigen::MatrixXd tunnel(2, 3);
      tunnel.row(0) << tunnel_start(0), tunnel_start(1), vins_altitude;
      tunnel.row(1) << tunnel_end(0), tunnel_end(1), vins_altitude;      
      waypointsVisual(tunnel);
      tunnel_traj = trajGeneration(tunnel);

      yaw_adjust_finished = true;
      state = FLYING_VIA_TUNNEL;
      const char* logMessage = "STATE: FLYING_VIA_TUNNEL";
      printf("\033[1;32m%s\033[0m\n", logMessage);
    }
  }

  if (state == FLYING_VIA_TUNNEL) {
    static ros::Time tunnel_time = ros::Time::now();
    static bool tunnel_init = true;
    ControlParse(tunnel_traj, tunnel_time, tunnel_init, tunnel_finished, 1);
    tunnel_init = false; 
  }

  if (state == STATE::SEARCHING_MAZE) {
    // if (!first_search_maze_finished) {
    //   static ros::Time first_search_maze_start_time = ros::Time::now();
    //   static bool first_search_maze_init = true;
    //   ControlParse(cricle2apriltag_traj, first_search_maze_start_time, first_search_maze_init, first_search_maze_finished, 1);
    //   first_search_maze_init = false;  
    // }else if (!second_search_maze_finished && first_search_maze_finished) {
    //   static ros::Time second_search_maze_start_time = ros::Time::now();
    //   static bool second_search_maze_init = true;
    //   ControlParse(cricle2apriltag_traj, second_search_maze_start_time, second_search_maze_init, second_search_maze_finished, 1);
    //   second_search_maze_init = false;        
    // }

      static ros::Time fly2tunnel_start_time = ros::Time::now();
      static bool fly2tunnel_init = true;
      ControlParse(fly2tunnel_start_traj, fly2tunnel_start_time, fly2tunnel_init, fly2tunnel_finished, 1);
      fly2tunnel_init = false; 
  }

  if (state == STATE::PASSING_CIRCLES) {

    static Eigen::Vector3d pass_circle_pos; 

    //如果暂时找不到圆，就先搜索

    static bool move_method = false;

    static double detla_x, detla_y, detla_z = 0;

    static Eigen::Vector3d ori = Vector3d::Zero();
    static Eigen::Vector3d forward = Vector3d::Zero();

    if (!have_circle && !toward_circle_finished && has_odom) {

      static Eigen::Vector3d start_serach_pos = odom_pos;
      static Eigen::Vector3d left_dis(0, 1.5, 0);
      static Eigen::Vector3d right_dis(0, -1.5, 0);
      static Eigen::Vector3d left_pos = start_serach_pos + odom_q * left_dis;
      static Eigen::Vector3d right_pos = start_serach_pos + odom_q * right_dis;

      //0-左   1-右
      static bool turn_ori = false;

      if (have_circle_cnt < 1) {
        if (Distance_of_waypoints(odom_pos, start_serach_pos) < 1.5 && !turn_ori) {
          cout << Distance_of_waypoints(odom_pos, left_pos) << endl;
          Eigen::Vector3d left(0, 0.004, 0);
          BodyMoveControlParse(left, planning_circle_altitude);
        }else if (!turn_ori) {
          turn_ori = true;
        }
        else if (turn_ori && Distance_of_waypoints(odom_pos, left_pos) < 2.9) {
          Eigen::Vector3d right(0, -0.004, 0);
          BodyMoveControlParse(right, planning_circle_altitude);   
        }else {
          state = LANDING;
        }
      } else {
        if (move_method) {
          BodyForwardControlParse(ori);

        }else {
          BodyForwardControlParse(forward);
        }

      }
    
    }else if (have_circle && !toward_circle_finished && has_odom) {

      int diff = circle_pos.second.x - 320;
      //0 - 1阶段  1 - 2阶段
      if (!move_method) {
        if (abs(diff) > 50) {
          detla_y = LimitOutput(-0.0064 * diff, 0.35);
          detla_x = 0;
          forward << 0, detla_y, 0;
          BodyForwardControlParse(forward);
        }else {
          ROS_WARN("switch to the second priod");
          move_method = true;
        }
      }

      if (move_method) {

        if (circle_pos.second.z > planning_circle_radius) {

          //先让飞机稳住
          static Eigen::Vector3d cur_uav_pos = odom_pos;
          posiCmd.header.stamp = ros::Time::now();
          posiCmd.position.x = cur_uav_pos(0);
          posiCmd.position.y = cur_uav_pos(1);
          posiCmd.position.z = planning_circle_altitude;
          posiCmd.velocity.x = 0;
          posiCmd.velocity.y = 0;
          posiCmd.velocity.z = 0;
          posiCmd.yaw = last_yaw_;
          posiCmd.yaw_dot = 0;
          last_yaw_ = posiCmd.yaw;
          controlCmd_pub.publish(posiCmd);

          ros::Rate ghgh(100);
          for (int i = 0; i < 200; i++) {
            ghgh.sleep();
          }

          //规划一条飞往圆心的轨迹
          Eigen::MatrixXd toward_circle(2, 3);
          toward_circle.row(0) << cur_uav_pos(0), circle_world_pos(1), planning_circle_altitude;
          toward_circle.row(1) << circle_world_pos(0), circle_world_pos(1), planning_circle_altitude;
          //第四关圆飞向终点的第一个路标点
          cricle2apriltag << circle_world_pos(0), circle_world_pos(1) + 5, planning_circle_altitude;      
          waypointsVisual(toward_circle);
          toward_circle_traj = trajGeneration(toward_circle);

  

          //跟踪给定轨迹
          ros::Rate circle_sleep(100);
          while (!toward_circle_finished) {
            static ros::Time toward_circle_time = ros::Time::now();
            static bool toward_circle_init = true;
            ControlParse(toward_circle_traj, toward_circle_time, toward_circle_init, toward_circle_finished, 0);
            toward_circle_init = false;
            circle_sleep.sleep();
          }
        }

        if ( !toward_circle_finished ) {
          detla_y = LimitOutput(-0.003 * diff, 0.4);
          detla_x = sqrt(0.4 * 0.4 - detla_y * detla_y) * 0.75;
          ori << detla_x, detla_y, detla_z;
          BodyForwardControlParse(ori);
        }
      }
    }
  }


  if (state == STATE::SEARCHING_APRILTAG) {
    static bool plan_finished = false;

    //该阶段是圆到其右侧的坐标点
    if (!cricle2apriltag_finished) {
      static ros::Time cricle2apriltag_start_time = ros::Time::now();
      static bool fcricle2apriltag_init = true;
      ControlParse(cricle2apriltag_traj, cricle2apriltag_start_time, fcricle2apriltag_init, cricle2apriltag_finished, 1);
      fcricle2apriltag_init = false;      
    
    //该阶段是在往预设二维码终点搜索
    }else if (cricle2apriltag_finished && !fly2apriltag_finished && !apriltag_detected) {
      static ros::Time fly2apriltag_start_time = ros::Time::now();
      static bool fly2apriltag_init = true;
      ControlParse(fly2apriltag_traj, fly2apriltag_start_time, fly2apriltag_init, fly2apriltag_finished, 1);
      fly2apriltag_init = false;
      
    //如果找到了二维码直接悬停，并且规划到二维码的轨迹
    }else if (apriltag_detected && !plan_finished) {
      ros::Rate jik(100);
      for (int i = 0; i < 200; i++) {
        posiCmd.header.stamp = ros::Time::now();
        posiCmd.position.x = odom_pos(0);
        posiCmd.position.y = odom_pos(1);
        posiCmd.position.z = searching_apriltag_altitude;
        posiCmd.velocity.x = 0;
        posiCmd.velocity.y = 0;
        posiCmd.velocity.z = 0;
        posiCmd.yaw = last_yaw_;
        posiCmd.yaw_dot = 0;
        last_yaw_ = posiCmd.yaw;
        controlCmd_pub.publish(posiCmd);
        jik.sleep();
      }

      state = LANDING;

      Eigen::MatrixXd fly2land(2, 3);
      Eigen::Vector3d fly2land_start(odom_pos(0), odom_pos(1), searching_apriltag_altitude);
      fly2land.row(0) = fly2land_start;
      fly2land.row(1) << apriltag_pos(0), apriltag_pos(1), searching_apriltag_altitude;      
      waypointsVisual(fly2land);
      fly2land_traj = trajGeneration(fly2land);
      plan_finished = true;

    //如果规划完了就执行到二维码终点的轨迹
    }else if (plan_finished && !fly2land_finished){
      static ros::Time fly2land_start_time = ros::Time::now();
      static bool fly2land_init = true;
      ControlParse(fly2land_traj, fly2land_start_time, fly2land_init, fly2land_finished, 0);
      fly2land_init = false;
    
    //如果没找到二维码或者已经飞到了二维码的地方就降落
    }else if ((!apriltag_detected && fly2apriltag_finished) || fly2land_finished) {
      state = LANDING;
      const char* land_log = "STATE: LANDING";
      printf("\033[1;32m%s\033[0m\n", land_log);
    }

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
  //只有在迷宫关卡才会对ego-planner的规划进行转发
  // if (state == SEARCHING_MAZE && !circle_adjust_yaw) {
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







