#include "console/console.hpp"

TRAJECTORY_GENERATOR_WAYPOINT::TRAJECTORY_GENERATOR_WAYPOINT(){}
TRAJECTORY_GENERATOR_WAYPOINT::~TRAJECTORY_GENERATOR_WAYPOINT(){}

//define factorial function, input i, output i!
int TRAJECTORY_GENERATOR_WAYPOINT::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::MatrixXd TRAJECTORY_GENERATOR_WAYPOINT::PolyQPGeneration(const int d_order,           // the order of derivative
                                                              const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
                                                              const Eigen::MatrixXd &Vel,  // boundary velocity
                                                              const Eigen::MatrixXd &Acc,  // boundary acceleration
                                                              const Eigen::VectorXd &Time) // time allocation in each segment
{
    int p_order = 2 * d_order - 1;      // the order of polynomial
    int p_num1d = p_order + 1;          // the number of Coefficients in each segment
    int seg_num = Time.size();          // the number of segments

    MatrixXd PolyCoeff = MatrixXd::Zero(seg_num, 3 * p_num1d);     // position(x,y,z), so we need (3 * p_num1d) coefficients

    VectorXd Px(p_num1d * seg_num);     // coefficients in each axis
    VectorXd Py(p_num1d * seg_num);
    VectorXd Pz(p_num1d * seg_num);

    // enforce initial and final position,velocity and accleration, for higher order derivatives, just assume them be 0
    MatrixXd StartState(d_order, 3);
    MatrixXd EndState(d_order, 3);
    StartState.row(0) = Path.row(0);
    StartState.row(1) = Vel.row(0);
    StartState.row(2) = Acc.row(0);
    EndState.row(0) = Path.row((Path.rows()-1));
    EndState.row(1) = Vel.row(1);
    EndState.row(2) = Acc.row(1);
    if(d_order == 4)
    {
        StartState.row(3) = VectorXd::Zero(3);  // jerk
        EndState.row(3) = VectorXd::Zero(3); 
    }
    // cout << " StartState = " << endl;
    // cout << StartState << endl;
    // cout << " EndState = " << endl;
    // cout << EndState << endl;


    // MatrixXd DiffMat = MatrixXd::Zero(p_num1d, p_num1d);
    // for(int i = 0; i < DiffMat.rows()-1; i++)
    //     DiffMat(i,i+1) = i+1;
    // cout << " DiffMat = " << endl;
    // cout << DiffMat << endl;

    // MatrixXd A = MatrixXd::Identity(p_num1d, p_num1d);
    // for(int i = 0; i < d_order; i++)   
    // {
    //     A *= DiffMat;                // coefficients of n-order derivative
    //     cout << " A = " << endl;
    //     cout << A << endl;
    // } 
 

    _Q = MatrixXd::Zero(p_num1d * seg_num, p_num1d * seg_num);
    _M = MatrixXd::Zero(p_num1d * seg_num, p_num1d * seg_num);
    _Ct = MatrixXd::Zero(2 * d_order * seg_num, d_order * (seg_num + 1));

    for(int seg_index = 0; seg_index < seg_num; seg_index++)
    {
        // calculate Matrix Q
        _Q.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getQ(p_num1d, d_order, Time, seg_index);
        // calculate Matrix M
        _M.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getM(p_num1d, d_order, Time, seg_index);
    }
    // calculate Matrix Ct
    _Ct = getCt(seg_num, d_order);

    // cout << " Q = " << endl;
    // cout << _Q << endl;
    // cout << " M = " << endl;
    // cout << _M << endl;
    // cout << " Ct = " << endl;
    // cout << _Ct << endl;

    Px = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(0), StartState.col(0), EndState.col(0), seg_num, d_order);
    Py = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(1), StartState.col(1), EndState.col(1), seg_num, d_order);
    Pz = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(2), StartState.col(2), EndState.col(2), seg_num, d_order);
    // cout << " Px = " << endl;
    // cout << Px << endl;
    // cout << " Py = " << endl;
    // cout << Py << endl;
    // cout << " Pz = " << endl;
    // cout << Pz << endl;

    for(int i = 0; i < seg_num; i++)
    {
        PolyCoeff.row(i).segment(0, p_num1d) = Px.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(p_num1d, p_num1d) = Py.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(2*p_num1d, p_num1d) = Pz.segment(p_num1d*i, p_num1d);
    }
    // cout << " PolyCoeff = " << endl;
    // cout << PolyCoeff << endl;

    return PolyCoeff;
}

Eigen::MatrixXd TRAJECTORY_GENERATOR_WAYPOINT::getQ(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    // calculate Matrix Q_k of the seg_index-th segment
    MatrixXd Q_k = MatrixXd::Zero(p_num1d, p_num1d);
    for (int i = 0; i < p_num1d; i++)
    {
        for (int j = 0; j < p_num1d; j++)
        {
            if (i >= p_num1d - d_order && j >= p_num1d - d_order)
            {
                Q_k(i, j) = (Factorial(i) / Factorial(i - d_order)) * ((Factorial(j) / Factorial(j - d_order))) /
                            (i + j - 2 * d_order + 1) * pow(Time(seg_index), (i + j - 2 * d_order + 1)); // Q of one segment
            }
        }
    }
    // cout << " Q_k = " << endl;
    // cout << Q_k << endl;

    return Q_k;
}


Eigen::MatrixXd TRAJECTORY_GENERATOR_WAYPOINT::getM(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    MatrixXd M_k = MatrixXd::Zero(p_num1d, p_num1d);
    VectorXd t_pow = VectorXd::Zero(p_num1d);
    for(int i = 0; i < p_num1d; i++)
    {
        t_pow(i) = pow(Time(seg_index),i);
    }
    // cout << "t_pow = " << endl;
    // cout << t_pow << endl;

    if(p_num1d == 6)        // minimum jerk
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,
               1, t_pow(1),   t_pow(2),   t_pow(3),    t_pow(4),    t_pow(5),
               0,     1   , 2*t_pow(1), 3*t_pow(2),  4*t_pow(3),  5*t_pow(4),
               0,     0   ,     2     , 6*t_pow(1), 12*t_pow(2), 20*t_pow(3);
    }
    else if(p_num1d == 8)   // minimum snap
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     0     ,     6     ,      0     ,      0     ,      0     ,      0     ,
               1, t_pow(1),   t_pow(2),   t_pow(3),    t_pow(4),    t_pow(5),    t_pow(6),    t_pow(7),
               0,     1   , 2*t_pow(1), 3*t_pow(2),  4*t_pow(3),  5*t_pow(4),  6*t_pow(5),  7*t_pow(6),
               0,     0   ,     2     , 6*t_pow(1), 12*t_pow(2), 20*t_pow(3), 30*t_pow(4), 42*t_pow(5),
               0,     0   ,     0     ,     6     , 24*t_pow(1), 60*t_pow(2),120*t_pow(3),210*t_pow(4);
    }
    // cout << "M_k = " << endl;
    // cout << M_k << endl;

    return M_k;
}

Eigen::MatrixXd TRAJECTORY_GENERATOR_WAYPOINT::getCt(const int seg_num, const int d_order)
{
    int d_num = 2 * d_order * seg_num;
    int df_and_dp_num = d_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * d_order + mid_waypts_num;
    // int dp_num = (d_order - 1) * mid_waypts_num;

    Eigen::MatrixXd Ct = MatrixXd::Zero(d_num, df_and_dp_num);
    
    // Ct for the first segment: pos,vel,acc,(jerk)
    Ct.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);
    // Ct for the last segment: pos,vel,acc,(jerk)
    Ct.block(d_num - d_order, df_num - d_order, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    for(int mid_waypts_index = 0; mid_waypts_index < mid_waypts_num; mid_waypts_index++)
    {
        // Ct for middle waypoints: pos
        Ct(d_order+2*d_order*mid_waypts_index, d_order+mid_waypts_index) = 1;
        Ct(d_order+(d_order+2*d_order*mid_waypts_index), d_order+mid_waypts_index) = 1;

        // Ct for middle waypoints: vel
        Ct(d_order+1+2*d_order*mid_waypts_index, df_num+(d_order-1)*mid_waypts_index) = 1;
        Ct(d_order+(d_order+1+2*d_order*mid_waypts_index), df_num+(d_order-1)*mid_waypts_index) = 1;

        // Ct for middle waypoints: acc
        Ct(d_order+2+2*d_order*mid_waypts_index, (df_num+1)+(d_order-1)*mid_waypts_index) = 1;
        Ct(d_order+(d_order+2+2*d_order*mid_waypts_index), (df_num+1)+(d_order-1)*mid_waypts_index) = 1;

        if(d_order == 4)  // minimum snap
        {
            // Ct for middle waypoints: jerk
            Ct(d_order+3+2*d_order*mid_waypts_index, (df_num+2)+(d_order-1)*mid_waypts_index) = 1;
            Ct(d_order+(d_order+3+2*d_order*mid_waypts_index), (df_num+2)+(d_order-1)*mid_waypts_index) = 1;   
        }
    }
    // cout << "Ct = " << endl;
    // cout << Ct << endl;

    return Ct;
}

Eigen::VectorXd TRAJECTORY_GENERATOR_WAYPOINT::closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                                                  const Eigen::MatrixXd &M,
                                                                  const Eigen::MatrixXd &Ct,
                                                                  const Eigen::VectorXd &WayPoints1D,
                                                                  const Eigen::VectorXd &StartState1D,
                                                                  const Eigen::VectorXd &EndState1D,
                                                                  const int seg_num,
                                                                  const int d_order)
{
    int df_and_dp_num = d_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * d_order + mid_waypts_num;
    int dp_num = (d_order - 1) * mid_waypts_num;

    Eigen::MatrixXd C = Ct.transpose();
    Eigen::MatrixXd M_inv = M.inverse();
    Eigen::MatrixXd M_inv_tran = M_inv.transpose();

    Eigen::MatrixXd R = C * M_inv_tran * Q * M_inv * Ct;
    Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
    Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);

    // compute dF
    Eigen::VectorXd dF(df_num);
    dF.head(d_order) = StartState1D;    // start state: pos,vel,acc,(jerk)
    dF.segment(d_order, mid_waypts_num) = WayPoints1D.segment(1,WayPoints1D.rows()-2);  // middle waypoints: pos
    dF.tail(d_order) = EndState1D;      // end state: pos,vel,acc,(jerk)
    // cout << "dF = " << endl;
    // cout << dF << endl;
    
    Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;   // closed-form solution of Unconstrained quadratic programming

    Eigen::VectorXd dF_and_dP(df_and_dp_num);
    dF_and_dP << dF, dP;
    Eigen::VectorXd PolyCoeff1D = M_inv * Ct * dF_and_dP;   // all coefficients of one segment

    return PolyCoeff1D;
}


Eigen::VectorXd CONSOLE::timeAllocation(Eigen::MatrixXd& Path){
    VectorXd time(Path.rows() - 1);
    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {

        if (state == SEARCHING_MAZE) {
            double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
            double x1 = 2.0 * 2.0 / (2 * _Acc); 
            double x2 = distance - 2 * x1;
            double t1 = 2.0 / _Acc;
            double t2 = x2 / 2.0;
            time(i) = 2 * t1 + t2;            
        }else if (state == SEARCHING_APRILTAG){
            double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
            double x1 = 1.5 * 1.5 / (2 * _Acc); 
            double x2 = distance - 2 * x1;
            double t1 = 1.5 / _Acc;
            double t2 = x2 / 1.5;
            time(i) = 2 * t1 + t2;  
        }else {
            double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
            double x1 = _Vel * _Vel / (2 * _Acc); 
            double x2 = distance - 2 * x1;
            double t1 = _Vel / _Acc;
            double t2 = x2 / _Vel;
            time(i) = 2 * t1 + t2;
        }

    }
    return time;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>  CONSOLE::getTrajInfo(const MiniSnapTraj& traj, double time){
    int     traj_id;
    double  t;
    //判断当前时间处于哪段轨迹当中
    for (int i = 0; i < traj.traj_nums; i++){
        if (time <= traj.poly_time(i)){
            traj_id = i;
            t = (i == 0 ? time : time - traj.poly_time(i - 1));
            break;
        } 
    }
    Vector3d position, velocity;
    for (int dim = 0; dim < 3; dim++){
        VectorXd coeff      = (traj.poly_coeff.row(traj_id)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd posi_time  = VectorXd::Zero( _poly_num1D );
        VectorXd vel_time   = VectorXd::Zero( _poly_num1D );

        posi_time(0) = 1.0;
        vel_time(0)  = std::numeric_limits<double>::min();
        
        for (int j = 1; j < _poly_num1D; j++){
            if (j == 1){
                posi_time(j) = pow(t, j);
                vel_time(j)  = 1;
            }else{
                posi_time(j) = pow(t, j);
                vel_time(j)  = j * pow(t, j - 1);                
            }
        }
        position(dim) = coeff.dot(posi_time);
        velocity(dim) = coeff.dot(vel_time);
    }
    return std::pair<Eigen::Vector3d, Eigen::Vector3d>(position, velocity);
}

MiniSnapTraj CONSOLE::trajGeneration(Eigen::MatrixXd& path){
    TRAJECTORY_GENERATOR_WAYPOINT trajectoryGeneratorWaypoint;
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;
    // use "trapezoidal velocity" time allocation
    ros::Time time_start = ros::Time::now();
    auto _polyTime  = timeAllocation(path);
    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    auto _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
    MiniSnapTraj Traj = MiniSnapTraj(time_start, _polyTime, _polyCoeff);
    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0); 
    ROS_WARN("The duration of trajectory is %f s", Traj.time_duration); 
    //轨迹可视化
    getVisual(Traj);
    return Traj;
}


void CONSOLE::getVisual(MiniSnapTraj& trajectory){
    double traj_len = 0.0;
    int count = 1;
    Eigen::Vector3d cur, pre, vel;
    cur.setZero();
    pre.setZero();
    geometry_msgs::PoseStamped poses;
    trajPath.header.frame_id = poses.header.frame_id = "world";
    trajPath.header.stamp    = poses.header.stamp    = ros::Time::now();
    poses.pose.orientation     = tf::createQuaternionMsgFromYaw(0.0);

    ros::Rate loop_rate(1000);
    for(double t = 0.0; t < trajectory.time_duration; t += 0.01, count++)   // go through each segment
    {   
        auto info = getTrajInfo(trajectory, t);
        cur = info.first;
        vel = info.second;
        poses.pose.position.x = cur[0];
        poses.pose.position.y = cur[1];
        poses.pose.position.z = cur[2];
        trajPath.poses.push_back(poses);
        traj.push_back(cur);
        trajPath_pub.publish(trajPath);
        if (count % 1000 == 0) traj_len += (pre - cur).norm();
        pre = cur;
        loop_rate.sleep();
    }
    //发布轨迹到rviz进行可视化
    trajVisual_->displayTraj(traj);
}
