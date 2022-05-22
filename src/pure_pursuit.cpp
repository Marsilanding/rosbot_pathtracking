#define SAMPLE_RATE 10
#define SPEED 1

#include "ros/ros.h"
#include "math.h"
#include "pathtracking/GetPath.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cstdlib>


nav_msgs::Odometry currentPose;
float currentX, currentY;
float lookAhead = 1;

void updatePos(const nav_msgs::Odometry::ConstPtr& msg){
  currentPose = *msg;
  currentX = currentPose.pose.pose.position.x;
  currentY = currentPose.pose.pose.position.y;
  //ROS_INFO("Robot position \n\t x = %f \n\t y = %f", currentX, currentY);
}

int computeNearestWayPoint(float **path, int pathSize, float lookAhead)
{
  float distance, newDistance;
  // calcula la diferencia entre la distancia con el primer punto y la distancia lookAhead
  distance = abs(sqrt(pow((currentX - path[0][0]),2) - pow((currentY - path[0][1]),2)) - lookAhead);
  int closest = 0;
  for(int i = 1; i < pathSize; i++)
  {
    newDistance = abs(sqrt(pow((currentX - path[i][0]),2) - pow((currentY - path[i][1]),2)) - lookAhead);
    // Si el nuevo punto esta mas cerca, reemplaza al anterior
    if(newDistance < distance)
    {
      distance = newDistance;
      closest = i;
    }
  }

  return closest;
}

float computeAlpha(float *goal, float lookAhead)
{
  float alpha = (2*abs(goal[1]-currentY))/lookAhead;

  return alpha;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  if (argc != 3)
  {
    ROS_INFO("Provide x and y coordinates of goal");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient path_client = n.serviceClient<pathtracking::GetPath>("get_path");
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, updatePos);
  pathtracking::GetPath srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);

  int pathSize;
  float** path;

  if (path_client.call(srv))
  {
    ROS_INFO("Path received");
    int pathSize = srv.response.x.size(); // Path size
    float path[pathSize][2];

    for(int i = 0; i < pathSize; i++)
    {
      path[i][0] = srv.response.x[i];
      path[i][1] = srv.response.y[i];
    }
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  ros::Rate r(SAMPLE_RATE); // 10 hz
  while(ros::ok())
  {
    int closest = computeNearestWayPoint(path, pathSize, lookAhead);
    float alpha = computeAlpha(path[closest], lookAhead);

    // Publish the command to change orientation
    geometry_msgs::Twist command;

    command.angular.z = -alpha;
    command.angular.x = 0;
    command.angular.y = 0;
    command.linear.x = SPEED;
    command.linear.y = 0;
    command.linear.z = 0;

    cmd_pub.publish(command);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}