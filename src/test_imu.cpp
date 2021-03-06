
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
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

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
const int MatrixSize = 50;

void testEigen() {
  Eigen::Matrix<float, 2, 3> matrix_23;
  Eigen::Vector3d vec_3d;
  Eigen::Matrix3d matrix_33d;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
  Eigen::MatrixXd matrix_xd;
  matrix_23 << 1, 0, 0, 0, 1, 0;
  cout << matrix_23 << endl;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      cout << matrix_23(i, j) << endl;
    }
  }

  vec_3d << 1, 0, 0;
  Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * vec_3d;
  cout << result << endl;
  // Eigen::Matrix<double, 3, 1> result_error_dimention =
  //     matrix_23.cast<double>() * vec_3d;
  matrix_33d = Eigen::Matrix3d::Random();
  cout << "random matrix33d: " << matrix_33d << endl;
  cout << matrix_33d.transpose() << endl;
  cout << matrix_33d.inverse() << endl;
  cout << matrix_33d.sum() << endl;
  cout << matrix_33d.trace() << endl;
  cout << 10 * matrix_33d << endl;
  cout << matrix_33d.determinant() << endl;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(
      matrix_33d.transpose() * matrix_33d);
  cout << "eigen value: " << eigen_solver.eigenvalues() << endl;
  cout << "eigen vectors: " << eigen_solver.eigenvectors() << endl;

  Eigen::Matrix<double, MatrixSize, MatrixSize> matrix_NN;
  matrix_NN = Eigen::MatrixXd::Random(MatrixSize, MatrixSize);
  Eigen::Matrix<double, MatrixSize, 1> v_Nd;
  v_Nd = Eigen::MatrixXd::Random(MatrixSize, 1);

  clock_t time_stt = clock();
  Eigen::Matrix<double, MatrixSize, 1> x = matrix_NN.inverse() * v_Nd;
  cout << "just inverse matrix cost "
       << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;

  time_stt = clock();
  x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
  cout << "time use in Qr composition is "
       << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;

  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
  cout << rotation_vector.matrix() << endl;
  cout << "rotation matrix \n" << rotation_vector.matrix() << endl;
  rotation_matrix = rotation_vector.toRotationMatrix();
  Eigen::Vector3d v(1, 0, 0);
  Eigen::Vector3d v_rotated = rotation_vector * v;
  cout << "v rotated " << v_rotated.transpose() << endl;

  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  cout << "y p r " << euler_angles.transpose() << endl;

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(rotation_vector);
  T.pretranslate(Eigen::Vector3d(1, 3, 4));
  cout << "T \n " << T.matrix() << endl;
  Eigen::Vector3d v_transformed = T * v;
  cout << "v transformed " << v_transformed.transpose() << endl;

  Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
  cout << "quaternion \n " << q.coeffs() << endl;

  q = Eigen::Quaterniond(rotation_matrix);
  cout << "quaternion = \n " << q.coeffs() << endl;

  v_rotated = q * v;
  cout << "1 0 0 -> " << v_rotated.transpose() << endl;

  return;
  // Eigen::Matrix<double, MatrixSize, 1>
}

vector<sensor_msgs::Imu> imuBuffer;
vector<sensor_msgs::Imu> batchImuBuffer;
double last_time;
bool first_msg = true;
bool use_dynamic_initialization = false;
bool use_static_initialization = false;
int batch_num = 20;
bool checkStatic = false;

// state variables
Eigen::Quaterniond q_wi;
Eigen::Vector3d p_0, v_0, ba_0, bg_0, acc_0, gyro_0;
Eigen::Quaterniond q_0;
Eigen::Vector3d gravity_w;

ros::Publisher odom_pub;

void predict(double dt, const sensor_msgs::Imu &imu_msg) {
  Eigen::Vector3d m_acc, m_gyro;
  tf::vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);
  tf::vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
  Eigen::Vector3d un_acc_0 = q_0 * (acc_0 - ba_0) + gravity_w;
  Eigen::Vector3d un_gyro = 0.5 * (gyro_0 + m_gyro) - bg_0;
  Eigen::Quaterniond dq_tmp;
  dq_tmp.x() = un_gyro.x() * dt / 2.0;
  dq_tmp.y() = un_gyro.y() * dt / 2.0;
  dq_tmp.z() = un_gyro.z() * dt / 2.0;
  dq_tmp.w() = 1.0;
  dq_tmp.normalize();
  q_0 = q_0 * dq_tmp;
  Eigen::Vector3d un_acc_1 = q_0 * (m_acc - ba_0) + gravity_w;
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  p_0 = p_0 + dt * v_0 + 0.5 * un_acc * dt * dt;
  v_0 = v_0 + dt * un_acc;
  acc_0 = m_acc;
  gyro_0 = m_gyro;
}

void publish(const sensor_msgs::Imu &imu_msg) {
  nav_msgs::Odometry odometry;
  odometry.header = imu_msg.header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = p_0.x();
  odometry.pose.pose.position.y = p_0.y();
  odometry.pose.pose.position.z = p_0.z();
  odometry.pose.pose.orientation.x = q_0.x();
  odometry.pose.pose.orientation.y = q_0.y();
  odometry.pose.pose.orientation.z = q_0.z();
  odometry.pose.pose.orientation.w = q_0.w();
  odometry.twist.twist.linear.x = v_0.x();
  odometry.twist.twist.linear.y = v_0.y();
  odometry.twist.twist.linear.z = v_0.z();
  odom_pub.publish(odometry);
}

bool staticInitialize() {
  Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_acc, m_gyro;

  for (auto imu_msg : imuBuffer) {
    tf::vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);
    tf::vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
    sum_acc += m_acc;
    sum_gyro += m_gyro;
  }
  bg_0 = sum_gyro / imuBuffer.size();
  cout << "bg_0 " << bg_0 << endl;
  Eigen::Vector3d gravity_imu = sum_acc / imuBuffer.size();

  double gravity_norm = gravity_imu.norm();
  gravity_w = Eigen::Vector3d(0.0, 0.0,
                              -gravity_norm);  // gravity or acc in world fram ?

  q_0 = Eigen::Quaterniond::FromTwoVectors(gravity_imu, -gravity_w);
  ba_0 = Eigen::Vector3d::Zero();
  p_0 = Eigen::Vector3d::Zero();
  v_0 = Eigen::Vector3d::Zero();
  acc_0 = m_acc;
  gyro_0 = m_gyro;
  return true;
}

bool batchImuProcess() {
  if (batchImuBuffer.size() == batch_num) {
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel_sum = Eigen::Vector3d::Zero();
    for (auto imu_msg : batchImuBuffer) {
      Eigen::Vector3d angular_velocity, linear_acc;
      tf::vectorMsgToEigen(imu_msg.angular_velocity, angular_velocity);
      tf::vectorMsgToEigen(imu_msg.linear_acceleration, linear_acc);
      acc_sum += linear_acc;
      angular_vel_sum += angular_velocity;
    }
    Eigen::Vector3d average_acc = acc_sum / batch_num;
    Eigen::Vector3d average_angular_vel = angular_vel_sum / batch_num;
    double acc_var = 0.0, angular_velocity_var;
    for (auto imu_msg : batchImuBuffer) {
      Eigen::Vector3d angular_velocity, linear_acc;
      tf::vectorMsgToEigen(imu_msg.angular_velocity, angular_velocity);
      tf::vectorMsgToEigen(imu_msg.linear_acceleration, linear_acc);
      acc_var += (linear_acc - average_acc).norm();
      angular_velocity_var += (angular_velocity - average_angular_vel).norm();
    }
    acc_var /= batch_num;
    angular_velocity_var /= batch_num;
    cout << "average acc " << average_acc << endl;
    cout << "average angular velocity: " << average_angular_vel << endl;
    ROS_INFO(" acc_val: %f ", acc_var);
    ROS_INFO(" angular_velocity_var: %f ", angular_velocity_var);

    if (acc_var > 0.5) {
      use_dynamic_initialization = true;
      checkStatic = true;
      // return dynamicInitialize();
      batchImuBuffer.clear();
    }
    if (acc_var < 0.5) {
      use_static_initialization = true;
      checkStatic = true;
      staticInitialize();

      batchImuBuffer.clear();
      imuBuffer.clear();
    }
  }
  ROS_INFO("batch size: %d ", batchImuBuffer.size());

  // imuBuffer.erase();
  return true;
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg) {
  // ROS_INFO(
  //     "received imu msg: timestamp: %d, acc_norm: %f,x: %f,y: %f,z: %f, "
  //     "gyro_norm: %f, %f, %f, %f ",
  //     msg->header.stamp.nsec,
  //     sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x +
  //          msg->linear_acceleration.y * msg->linear_acceleration.y +
  //          msg->linear_acceleration.z * msg->linear_acceleration.z),
  //     sqrt(msg->angular_velocity.x * msg->angular_velocity.x +
  //          msg->angular_velocity.y * msg->angular_velocity.y +
  //          msg->angular_velocity.z * msg->angular_velocity.z),
  //     msg->linear_acceleration.x, msg->linear_acceleration.y,
  //     msg->linear_acceleration.z, msg->angular_velocity.x,
  //     msg->angular_velocity.y, msg->angular_velocity.z);
  if (!checkStatic) {
    imuBuffer.push_back(*msg);
    last_time = msg->header.stamp.toSec();
  }
  if (imuBuffer.size() % batch_num == 0 && !checkStatic) {
    batchImuBuffer.push_back(*msg);
    batchImuProcess();

  } else {
    double dtime = msg->header.stamp.toSec() - last_time;
    last_time = msg->header.stamp.toSec();
    predict(dtime, *msg);
  }
  publish(*msg);

  ROS_INFO("buffer size: %d", imuBuffer.size());
  return;
}

int main(int argc, char **argv) {
  testEigen();

  ros::init(argc, argv, "imu_listener");

  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 1000);
  ros::Subscriber sub = n.subscribe("/imu0", 1000, imuCallback);

  ros::spin();

  return 0;
}
