#include <odom_fusion/odom_fusion.h>


void ODOM_FUSION::init(ros::NodeHandle& nh){

    odom_fusion_pub = nh.advertise<nav_msgs::Odometry>("/fusion/odom", 50);
    odom_sub        = nh.subscribe("odom", 100, &ODOM_FUSION::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    console_state_sub = nh.subscribe("/console/state", 100, &ODOM_FUSION::StateCallback, this, ros::TransportHints().tcpNoDelay());
    std::thread rtk_thread(&ODOM_FUSION::readLaser, this);
    rtk_thread.detach();                                            //分离线程在后台运行
}
void ODOM_FUSION::StateCallback(const console::ConsoleStateConstPtr msg) {
    state = msg->state;
}


void ODOM_FUSION::OdomCallback(const nav_msgs::OdometryConstPtr msg){
    if (have_altitude){

        static int hh = 0;

        if (hh % 100 == 0) cout << "current laser height: " << altitude << endl;
        hh++;

        static double pre_alt = altitude;
        if (altitude == 0) altitude = pre_alt;
        pre_alt = altitude;


        static bool flag = true;
        if (flag){
            ROS_WARN("Get Altitude!");
            flag = false;
        }
        odom_data = *msg;
        static bool odom_error = false;
        //cout << "dis: " << altitude - odom_data.pose.pose.position.z << endl;
        //当VINS估计出现严重偏差的时候，激光测距模块会介入 ，防止无人机高度爬升
        if (abs(odom_data.pose.pose.position.z) > 10 && altitude > 0.5) {
            odom_error = true;
        }
        if (odom_error || !(state == console::ConsoleState::FLYING_VIA_TUNNEL ||  state == console::ConsoleState::WAIT_TAKE_OFF_FINISHED || state == console::ConsoleState::FLY_TO_STRAT) ) odom_data.pose.pose.position.z = altitude;
        odom_fusion_pub.publish(odom_data);
    }
}


void ODOM_FUSION::readLaser(){

    serial::Serial serial_port;
    try
    {
        // 打开串口设备
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial_port.setBytesize(serial::eightbits);  // 设置数据位为8
        serial_port.setStopbits(serial::stopbits_one);  // 设置停止位为1
        serial_port.setParity(serial::parity_none);  // 设置奇偶校验位为None
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open the serial port: " << e.what());
        return ;
    }

    if(serial_port.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    }

    //经过实测发现，前41个字节没有用，所以先读取进行过滤
    serial_port.read(41);       

    ros::Rate loop_time(100);
    while(ros::ok()){
        if(serial_port.available()){
            uint8_t buffer[9];
            size_t bytes_read = serial_port.read(buffer, 9);
            // 检查是否成功读取到完整的数据帧
            if (bytes_read != 9)
            {
                ROS_ERROR_STREAM("Failed to read complete data frame");
                continue;
            }
            if (buffer[0] == 0x59 && buffer[1] == 0x59)
            {
                // 解析距离值
                uint16_t distance = (buffer[3] << 8) | buffer[2];
                altitude = static_cast<float>(distance) / 100.0;  // 距离值以米为单位
                have_altitude = true;        
            }
        }

    }    
}