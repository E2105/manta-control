// This state estimator simply reads IMU orientation and a pressure sensor,
// and publishes orientation and z-axis position based on the readings.
// The velocity twist is always zero, and x- and y-axis position is always
// zero.

#ifndef VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H
#define VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"

class SimpleEstimator
{
  public:
    SimpleEstimator();
    void imuCallback(const sensor_msgs::Imu &msg);
    void pressureCallback(const sensor_msgs::FluidPressure &msg);
    void publish();
  private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_pressure_sub;
    ros::Publisher  m_state_pub;
    ros::Publisher  m_depth_pub;

    double m_atmospheric_pressure;
    double m_water_density;
    double m_gravitational_acceleration;

    const double c_pi = 3.141592653589793;

    nav_msgs::Odometry m_state;
    std_msgs::Float64 depth;
};

#endif  // VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H
