#include "ros/ros.h"
#include "underwater_path_planning/PathPlan.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_path_client");
  if (argc != 5)
  {
    ROS_INFO("usage: plan a path startx starty goalx goaly");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<underwater_path_planning::PathPlan>("plan_path");
  underwater_path_planning::PathPlan srv;
  srv.request.startX = atoll(argv[1]);
  srv.request.startY = atoll(argv[2]);
  srv.request.goalX = atoll(argv[3]);
  srv.request.goalY = atoll(argv[4]);
  if (client.call(srv))
  {
    ROS_INFO("Called!");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
