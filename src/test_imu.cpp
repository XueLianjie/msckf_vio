
#include <Eigen/QR>
#include <Eigen/SPQRSupport>
#include <Eigen/SVD>
#include <Eigen/SparseCore>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <queue>

#include <eigen_conversions/eigen_msg.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
vector<sensor_msgs::Imu> imuBuffer;
double last_time;

bool batchImuProcess()
{
    for (auto imu_msg : imuBuffer)
    {
        Eigen::Vector3d angular_velocity, linear_velocity;
        tf::vectorMsgToEigen(imu_msg.angular_velocity, angular_velocity);
        tf::vectorMsgToEigen(imu_msg.linear_acceleration, linear_velocity);
    }
    imuBuffer.clear();
    return true;
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    ROS_INFO("received imu msg: timestamp: %d, acc_norm: %f,x: %f,y: %f,z: %f, gyro_norm: %f, %f, %f, %f ",
             msg->header.stamp.nsec, sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x + msg->linear_acceleration.y * msg->linear_acceleration.y + msg->linear_acceleration.z * msg->linear_acceleration.z),
             sqrt(msg->angular_velocity.x * msg->angular_velocity.x + msg->angular_velocity.y * msg->angular_velocity.y + msg->angular_velocity.z * msg->angular_velocity.z),
             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    imuBuffer.push_back(*msg);
    double dtime = msg->header.stamp.toSec() - last_time;
    if (imuBuffer.size() == 10)
    {
        batchImuProcess();
    }

    ROS_INFO("buffer size: %d", imuBuffer.size());
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/imu0", 1000, imuCallback);

    ros::spin();

    return 0;
}
