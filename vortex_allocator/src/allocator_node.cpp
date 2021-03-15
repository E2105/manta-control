#include "ros/ros.h"

#include "vortex_allocator/allocator_ros.h"      // STANDARDIZED

int main(int argc, char **argv)
{
  ros::init(argc, argv, "allocator");
  ros::NodeHandle nh;
  Allocator allocator(nh);
  ros::spin();
  return 0;
}
