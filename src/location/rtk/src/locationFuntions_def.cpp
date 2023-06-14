/*
**单独写一个cpp是为了实现LOCATION类中一些功能函数，减少程序的冗余
*/

#include "rtk/location.h"
using namespace std;


void LOCATION::Quaternion2Euler(const geometry_msgs::Quaternion& q, Euler& euler){
    // roll (x-axis rotation)

    // 将四元数转换为Eigen的Quaternion类型
    Eigen::Quaterniond eigenQuaternion(q.w, q.x, q.y, q.z);
    // 将Eigen的Quaternion类型转换为旋转矩阵(Matrix3d)
    Eigen::Matrix3d rotationMatrix = eigenQuaternion.toRotationMatrix();

    euler.roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    euler.pitch = asin(-rotationMatrix(2, 0));
    euler.yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

    cout << "YAW   : " << euler.yaw << endl;
    cout << "PITCH : " << euler.pitch << endl;  
    cout << "ROLL  : " << euler.roll << endl; 

    cout << "---------------------------" << endl;         

}
