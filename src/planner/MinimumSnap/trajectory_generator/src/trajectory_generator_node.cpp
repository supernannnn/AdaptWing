#include "trajectory_generator/trajectory_generator.h"

nav_msgs::Odometry odom;




int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_generator_node");
    ros::NodeHandle nh("~");
    TRAJECTORY_GENERATOR trajectory_generator;
    trajectory_generator.init(nh);
    ros::spin();
    return 0;
}


void TRAJECTORY_GENERATOR::init(ros::NodeHandle& nh){
    nh.param("planning/vel", _Vel, 1.0);
    nh.param("planning/acc", _Acc, 1.0);
    nh.param("planning/dev_order", _dev_order, 3);  // the order of derivative, _dev_order = 3->minimum jerk, _dev_order = 4->minimum snap
    nh.param("planning/min_order", _min_order, 3);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);


    _poly_num1D = 2 * _dev_order;

    
    waypoints_sub       = nh.subscribe("waypoints", 1, &TRAJECTORY_GENERATOR::waypointsCallback, this);
    path_vis_pub        = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);
    trajectory_vis_pub  = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);


    _path_vis_pub = nh.advertise<nav_msgs::Path>("vis_path", 1);


}


void TRAJECTORY_GENERATOR::waypointsCallback(const nav_msgs::Path::ConstPtr &msg){
    

    _path_vis_pub.publish(msg);

    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)msg->poses.size(); k++)
    {
        Vector3d pt( msg->poses[k].pose.position.x, msg->poses[k].pose.position.y, msg->poses[k].pose.position.z);
        wp_list.push_back(pt);
        ROS_INFO("waypoint%d: (%f, %f, %f)", k+1, pt(0), pt(1), pt(2));
    }


    //add the original point
    MatrixXd waypoints(wp_list.size() + 1, 3);  
    waypoints.row(0) = _startPos;

    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];

    trajGeneration(waypoints);
}


void TRAJECTORY_GENERATOR::trajGeneration(Eigen::MatrixXd& path){
    ros::Time time_start = ros::Time::now();

    TRAJECTORY_GENERATOR_WAYPOINT trajectoryGeneratorWaypoint;
    
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;

    // use "trapezoidal velocity" time allocation
    _polyTime  = timeAllocation(path);


    for (int i = 0; i < _polyTime.size(); i++){
        cout << _polyTime[i] << endl;
    }

    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);

    visWayPointPath(path);    // visulize path
    visTrajectory( _polyCoeff, _polyTime);    // visulize trajectory    
}


VectorXd TRAJECTORY_GENERATOR::timeAllocation(Eigen::MatrixXd& Path){
    VectorXd time(Path.rows() - 1);
    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
        double x1 = _Vel * _Vel / (2 * _Acc); 
        double x2 = distance - 2 * x1;
        double t1 = _Vel / _Acc;
        double t2 = x2 / _Vel;
        time(i) = 2 * t1 + t2;
    }

    return time;
}


Eigen::Vector3d TRAJECTORY_GENERATOR::getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t){
    Vector3d ret;
    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
    }
    return ret;
}



void TRAJECTORY_GENERATOR::visWayPointPath(Eigen::MatrixXd& path)
{

    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "np_point";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);
      line_list.points.push_back(p);
    }

    path_vis_pub.publish(points);
    path_vis_pub.publish(line_list);  
}


void TRAJECTORY_GENERATOR::visTrajectory(Eigen::MatrixXd& polyCoeff, Eigen::VectorXd& time){
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;


    for(int i = 0; i < time.size(); i++ )   // go through each segment
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }
    ROS_WARN("Trajectory length is %f m", traj_len);
    trajectory_vis_pub.publish(_traj_vis);
}











