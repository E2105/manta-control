#ifndef VORTEX_CONTROLLER_CONTROLLER_ROS_H
#define VORTEX_CONTROLLER_CONTROLLER_ROS_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

// Message classes
#include <std_msgs/String.h>
#include <std_msgs/ByteMultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "vortex_msgs/Debug.h"

#include "vortex/eigen_typedefs.h"

#include "vortex_controller/VortexControllerConfig.h"
#include "vortex_controller/control_modes.h"
#include "vortex_controller/state.h"
#include "vortex_controller/setpoints.h"
#include "vortex_controller/quaternion_pd_controller.h"



class Controller
{
public:
  explicit Controller(ros::NodeHandle nh);
  void commandCallback(const geometry_msgs::Twist &msg);
  void modeCallback(const std_msgs::ByteMultiArray &msg); // NEW FOR BYTE MODE ARRAY
  void stateCallback(const nav_msgs::Odometry &msg);      // standardized
  void configCallback(const vortex_controller::VortexControllerConfig& config, uint32_t level);
  void spin();
private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_command_sub;
  ros::Subscriber m_mode_sub;        // STANDARDIZATION
  ros::Subscriber m_state_sub;
  ros::Publisher  m_wrench_pub;
  ros::Publisher  m_mode_pub;
  ros::Publisher  m_debug_pub;
  dynamic_reconfigure::Server<vortex_controller::VortexControllerConfig> m_dr_srv;

  ControlMode m_control_mode;
  int m_frequency;
  bool m_debug_mode = false;
  const double c_normalized_force_deadzone = 0.01;
  const double c_max_quat_norm_deviation = 0.1;

  enum PoseIndex { SURGE = 0, SWAY = 1, HEAVE = 2, ROLL = 3, PITCH = 4, YAW = 5 };
  enum EulerIndex { EULER_YAW = 0, EULER_PITCH = 1, EULER_ROLL = 2 };

  std::unique_ptr<State>                  m_state;
  std::unique_ptr<Setpoints>              m_setpoints;
  std::unique_ptr<QuaternionPdController> m_controller;

  ControlMode getControlMode(const std_msgs::ByteMultiArray &msg) const;            // STANDARDIZATION
  void initSetpoints();
  void resetSetpoints();
  void updateSetpoint(PoseIndex axis);
  void initPositionHoldController();
  bool healthyMotionMessage(const geometry_msgs::Twist &msg);
  bool healthyModeMessage(const std_msgs::ByteMultiArray &msg); // STANDARDIZATION
  void publishControlMode();
  void publishDebugMsg(const Eigen::Vector3d    &position_state,
                       const Eigen::Quaterniond &orientation_state,
                       const Eigen::Vector6d    &velocity_state,
                       const Eigen::Vector3d    &position_setpoint,
                       const Eigen::Quaterniond &orientation_setpoint);
  Eigen::Vector6d stayLevel(const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state);
  Eigen::Vector6d depthHold(const Eigen::Vector6d &tau_openloop,
                            const Eigen::Vector3d &position_state,
                            const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state,
                            const Eigen::Vector3d &position_setpoint);
  Eigen::Vector6d headingHold(const Eigen::Vector6d &tau_openloop,
                              const Eigen::Vector3d &position_state,
                              const Eigen::Quaterniond &orientation_state,
                              const Eigen::Vector6d &velocity_state,
                              const Eigen::Quaterniond &orientation_setpoint);
  Eigen::Vector6d feedbackControl(const Eigen::Vector3d &position_state,          // Added for semi-autonomy
                                  const Eigen::Quaterniond &orientation_state,
                                  const Eigen::Vector6d &velocity_state,
                                  const Eigen::Quaterniond &orientation_setpoint);
};

#endif  // VORTEX_CONTROLLER_CONTROLLER_ROS_H
