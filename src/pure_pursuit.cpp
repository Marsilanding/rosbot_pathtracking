#include "ros/ros.h"
#include "pathtracking/GetPath.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  if (argc != 3)
  {
    ROS_INFO("usage: pure_pursuit control algorithm");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pathtracking::GetPath>("get_path");
  pathtracking::GetPath srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Path received");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}