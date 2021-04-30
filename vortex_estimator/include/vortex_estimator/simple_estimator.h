// This state estimator simply reads IMU orientation and a pressure sensor,
// and publishes orientation and z-axis position based on the readings.
// The velocity twist is always zero, and x- and y-axis position is always
// zero.

#ifndef VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H
#define VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "vortex_msgs/Location.h"
#include <string>

class SimpleEstimator
{
  public:
    SimpleEstimator();
    void imuCallback(const sensor_msgs::Imu &msg);
    void pressureCallback(const std_msgs::Float64 &msg);
    void locationCallback(const vortex_msgs/Location &msg);
    void publish();
  private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_depth_sub;
    ros::Subscriber m_location_sub;
    ros::Publisher  m_state_pub;

    double rov_location_x;
    double rov_location_y;
    
    bool m_ned_frame = false;
    const double c_pi = 3.141592653589793;

    nav_msgs::Odometry m_state;
};

#endif  // VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H
