// This state estimator simply reads IMU orientation and a pressure sensor,
// and publishes orientation and z-axis position based on the readings.
// The velocity twist is always zero, and x- and y-axis position is always
// zero.

#ifndef VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H
#define VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "nav_msgs/Odometry.h"              // standardization
#include "vortex_msgs/RovState.h"

class SimpleEstimator
{
  public:
    SimpleEstimator();
    void imuCallback(const sensor_msgs::Imu &msg);
    void pressureCallback(const sensor_msgs::FluidPressure &msg);
    void positionCallback(const sensor_msgs::UwGPS &msg);   //Added for UW GPS
    void publish();
  private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_pressure_sub;
    ros::Subscriber m_position_sub;  //Added for UW GPS
    ros::Publisher  m_state_pub;

    double m_atmospheric_pressure;
    double m_water_density;
    double m_gravitational_acceleration;

    const double c_pi = 3.141592653589793;

    nav_msgs::Odometry m_state;                       //standardization
};

#endif  // VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H
