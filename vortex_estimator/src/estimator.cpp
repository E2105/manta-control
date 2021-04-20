#include "vortex_estimator/simple_estimator.h"
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

SimpleEstimator::SimpleEstimator()
{
  // Initiating subscribers and publishers
  m_imu_sub      = m_nh.subscribe("/imu/data", 1, &SimpleEstimator::imuCallback, this);
  m_depth_sub    = m_nh.subscribe("/sensors/depth", 1, &SimpleEstimator::pressureCallback, this);
  m_state_pub    = m_nh.advertise<nav_msgs::Odometry>("estimator/state", 1);

  // Initiating orientation in quaternions
  //  This is along the x-axs pointing forward from the body
  m_state.pose.pose.orientation.w = 1.0;
  m_state.pose.pose.orientation.x = 0.0;
  m_state.pose.pose.orientation.y = 0.0;
  m_state.pose.pose.orientation.z = 0.0;

  ROS_INFO("Estimator initialized.");
}

void SimpleEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
  // ENU to NED transformation
  // Swap X and Y, invert Z

  // Converting the quaternion representation to Euler angles
  Eigen::Quaterniond quat_imu;
  tf::quaternionMsgToEigen(msg.orientation, quat_imu);
  Eigen::Vector3d euler_imu = quat_imu.toRotationMatrix().eulerAngles(2, 1, 0);

  // Convert Euler angles to "NED":
  // X positive forward/"north", Y positive to the right/"east", Z positive down.
  Eigen::Vector3d euler_ned(-euler_imu(0), euler_imu(2) + c_pi, euler_imu(1) + c_pi);

  // Transform back to quaternion
  Eigen::Matrix3d R_ned;
  R_ned = Eigen::AngleAxisd(euler_ned(0), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(euler_ned(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(euler_ned(2), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond quat_ned;

  // Publish the orientation in quaternions
  tf::quaternionEigenToMsg(quat_ned, m_state.pose.pose.orientation);
  m_state.twist.twist.angular.z = -msg.angular_velocity.z;
  m_state_pub.publish(m_state);
}

void SimpleEstimator::pressureCallback(const std_msgs::Float64 &msg)
{
  const float depth_meters = msg.data;
  m_state.pose.pose.position.z = depth_meters;
  m_state_pub.publish(m_state);
}
