#include "guidance/los_guidance.h"

#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

// import config
// Line of Sight Guidance
LoSGuidance::LoSGuidance(double lookahead_distance,
                         double sphere_of_acceptance)
{

}


// Returns rotations needed for Line of Sight guidance.
Eigen::quaterniond LoSGuidance::getLoSRotation(const Eigen::Vector3d   &position,
                                               const Eigen::Vector6d   &velocity,   //Uses Vector6, only needs the first 3.
                                               const Eigen::Vector3d   &waypoint,
                                               const Eigen::Vector3d   &last_waypoint)
{
  Eigen::Vector2d e = LoSGuidance::errorVector(position, last_waypoint, waypoint);
  Eigen::Vector3d delta = waypoint - last_waypoint;

  Eigen::Quaterniond qPsi    = LoSGuidance::getQuaternion(e[0], lookahead_distance, 1, -1);
  Eigen::Quaterniond qBeta   = LosGuidance::getQuaternion(velocity[1], velocity[u], 1, 1);
  //&Eigen::Quaterniond qBetaV  = 
  Eigen::Quaterniond qGamma  = LoSGuidance::getQuaternion(delta[1], delta[0], 1, 1);

  //Eigen::Quaterniond qTheta  = LoSGuidance::getQuaternion(e[1], velocity[1], 0, 1);
  //Eigen::Quaterniond qAlpha  = LoSGuidance::getQuaternion(velocity[2], velocity[0], 0, 1);
  //Eigen::Quaterniond qAlphaP = LoSGuidance::getQuaternion(delta[0], delta[2], 0, 1);
  
  Eigen::Quaterniond q =  (qPsi).normalize() * (qBeta).normalize() * (qGamma).normalize();
  
  return Eigen::Quaterniond q;
}


// Returns error vector with y_e and z_e. X_e is not used. 
Eigen::Vector2d LoSGuidance::errorVector(const Eigen::Vector3d &position,
                                         const Eigen::Vector3d &waypoint,
                                         const Eigen::vector3d &last_waypoint)

{
  Eigen::Vector3d error = position - last_waypoint;
  Eigen::Vector3d wp_line = waypoint - last_waypoint;

  r = math::atan2(wp_line[1], wp_line[0]);
  a = math::atan2(-wp_line[2], wp_line[0]);


// Ignore xe
  ye = -error[0]*math::sin(r) + error[1]*math::cos(r);
  ze = error[2]*math::sin(a) + error[0]*math::cos(a);

  return Eigen::Vector2d(ye, ze);
}


// Gives a quaternion representation of a rotation based on e and delta values.
Eigen::Quaterniond LoSGuidance::getQuaternion(const double  &a,
                                              const double  &b,
                                              const bool    &horizontal,
                                              const int     &sgn)

{
  R = math::sqrt(a*a + b*b);

  w = math::sqrt((b + R)/2*R);
  i = 0;
  if (horizontal)
    j = 0;
    k = (sgn*a)/(2*R*w);
  else
    j = (sgn*a)/(2*R*w);
    k = 0;

  return Eigen::Quaterniond(w, i, j, k);
}


// Sphere of Acceptance. 
bool LoSGuidance::sphereOfAcceptance(const Eigen::Vector3d  &position,
                                     const Eigen::Vector3d  &waypoint)
{
  Eigen::Vector3d e = waypoint - position;

  return e[0]*e[0]+e[1]*e[1]+e[2]*e[2] < sphere_of_acceptance*sphere_of_acceptance;
}