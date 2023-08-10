import rospy

from geometry_msgs.msg import TwistStamped

rospy.init_node('twist_stamped_publisher')  # 初始化ROS节点
pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)  # 创建发布者

rate = rospy.Rate(10)  # 设置发布频率，这里是1Hz

while not rospy.is_shutdown():
    twist_msg = TwistStamped()
    # 填充您想要的TwistStamped消息内容
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = 'world'
    twist_msg.twist.linear.x = 0.3  # 设置线性速度
    twist_msg.twist.angular.z = 0  # 设置角速度
    
    pub.publish(twist_msg)  # 发布消息
    rate.sleep()  # 按照指定频率等待

    