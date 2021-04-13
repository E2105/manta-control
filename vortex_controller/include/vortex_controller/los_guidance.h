#pragma once

#include <Eigen/Dense>
#include "vortex/eigen_typedefs.h"

class LoSGuidance
{
public: // Include variables and functions used from class
  LoSGuidance(double lookahead_distance,
              double sphere_of_acceptance)


  Eigen::Quaterniond getLoSRotation(const Eigen::Vector3d   &position,
                                    const Eigen::Vector3d   &velocity,
                                    const Eigen::Vector3d   &waypoint,
                                    const Eigen::Vector3d   &last_waypoint);


private: // Include variables and functions only used in class

  Eigen::Vector2d errorVector(const Eigen::Vector3d &position,
                              const Eigen::Vector3d &waypoint,
                              const Eigen::vector3d &last_waypoint);


  Eigen::Quaterniond  getQuaternion(const double  &a,
                                    const double  &b,
                                    const bool    &horizontal,
                                    const int     &sgn);


  bool  sphereOfAcceptance(const Eigen::Vector3d  &position,
                           const Eigen::Vector3d  &waypoint);



};