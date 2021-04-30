#include "vortex_estimator/simple_estimator.h"
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <ros/console.h>

SimpleEstimator::SimpleEstimator()
{
  // Initiating subscribers and publishers
  m_imu_sub      = m_nh.subscribe("/imu/data", 1, &SimpleEstimator::imuCallback, this);
  m_depth_sub    = m_nh.subscribe("/sensors/depth", 1, &SimpleEstimator::pressureCallback, this);
  
  m_location_sub = m_nh.subscribe("/rov/location", 1, &SimpleEstimator::locationCallback, this);  // Location
  m_otter_location_sub = m_nh.subscribe("/otter/location", 1 &SimpleEstimator::otterCallback, this);
  
  m_state_pub    = m_nh.advertise<nav_msgs::Odometry>("estimator/state", 1);

  if (!m_nh.getParam("/physical/ned_frame", m_ned_frame))
    ROS_WARN("Failed to read parameter: /physical/ned_frame. Using default reference: ENU.");

  // Initiating position as orio
  m_state.pose.pose.position.x = 0.0;
  m_state.pose.pose.position.y = 0.0;
  m_state.pose.pose.position.z = 1.0;
  
  // Initiate local variable for rov location
  rov_location_x = 0.0;
  rov_location_y = 0.0;
  
  // Initiating orientation pointing north
  m_state.pose.pose.orientation.w = 1.0;
  m_state.pose.pose.orientation.x = 0.0;
  m_state.pose.pose.orientation.y = 0.0;
  m_state.pose.pose.orientation.z = 0.0;

  ROS_INFO("Estimator initialized.");
}

void SimpleEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
  // The Xsens MTi 30 IMU logs attitude in a ENU reference frame
  //  The paramter file decides between ENU and NED

  if (m_ned_frame == true) {
    // NED -> swap x & y, negate z

    // Orientation
    m_state.pose.pose.orientation.x = msg.orientation.y;
    m_state.pose.pose.orientation.y = msg.orientation.x;
    m_state.pose.pose.orientation.z = -msg.orientation.z;
    m_state.pose.pose.orientation.w = msg.orientation.w;

    // Angular velocity (m/s)
    m_state.twist.twist.angular.x   = msg.angular_velocity.y;
    m_state.twist.twist.angular.y   = msg.angular_velocity.x;
    m_state.twist.twist.angular.z   = -msg.angular_velocity.z;

    // Linear acceleration (m^2/s)
    m_state.twist.twist.linear.x    = msg.linear_acceleration.y;
    m_state.twist.twist.linear.y    = msg.linear_acceleration.x;
    m_state.twist.twist.linear.z    = -msg.linear_acceleration.z;

    // Publish NED
    m_state_pub.publish(m_state);

  } else {
    // ENU -> stay as is

    // Orientation (Quaternions)
    m_state.pose.pose.orientation.x = msg.orientation.x;
    m_state.pose.pose.orientation.y = msg.orientation.y;
    m_state.pose.pose.orientation.z = msg.orientation.z;
    m_state.pose.pose.orientation.w = msg.orientation.w;

    // Angular velocity (m/s)
    m_state.twist.twist.angular.x   = msg.angular_velocity.x;
    m_state.twist.twist.angular.y   = msg.angular_velocity.y;
    m_state.twist.twist.angular.z   = msg.angular_velocity.z;

    // Linear acceleration (m^2/s)
    m_state.twist.twist.linear.x    = msg.linear_acceleration.x;
    m_state.twist.twist.linear.y    = msg.linear_acceleration.y;
    m_state.twist.twist.linear.z    = msg.linear_acceleration.z;

    // Publish topic
    m_state_pub.publish(m_state);
  }
}

void SimpleEstimator::pressureCallback(const std_msgs::Float64 &msg)
{
  // By default, depth gets more positive with higher pressure
  // m_state.pose.pose.position.x = 0.0;
  // m_state.pose.pose.position.y = 0.0;

  const double depth_meter = -msg.data;
  m_state.pose.pose.position.z = depth_meter;
  m_state_pub.publish(m_state);
}

void SimpleEstimator::locationCallback(const vortex_msgs/Location &msg) // Location
{
  // Input from Otter, publish to Odometry
  rov_location_x = msg.latitude;
  rov_location_y = msg.longitude;
} 

void SimpleEstimator::otterCallback(const vortex_msgs/Location &msg)
{
  m_state.pose.pose.position.x = msg.latitude  - rov_location_x;
  m_state.pose.pose.position.y = msg.longitude - rov_location_y;
  m_state_pub.publish(m_state);
}
