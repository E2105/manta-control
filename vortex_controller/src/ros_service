#include "ros/ros.h"
#include "vortex_controller/controller_ros.h"

#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_setpoint_client");
  if (argc != 7)
  {
    ROS_INFO("usage: set_setpoint_client x y z w i j k");
    return 1;
  }
  Eigen::Vector3d position = [argv[0], argv[1], argv[2]];
  Eigen::Quaterniond orientation = [argv[3], argv[4], argv[5], argv[6]];
  
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<vortex_controller::setSetpoint>("set_setpoint");
  vortex_controller::setSetpoint srv;
  srv.request.position = atoll(position);
  srv.request.orientation = atoll(orientation); 
  
  if (client.call(srv))
  {
    ROS_INFO("Success!");
  }
  else
  {
    ROS_ERROR("Failure!");
    return 1;
  } 
  return 0;
}     
