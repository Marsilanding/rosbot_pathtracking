#define SAMPLE_RATE 10

#include "ros/ros.h"
#include "math.h"
#include "pathtracking/GetPath.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cstdlib>

#include <tf/tf.h>


nav_msgs::Odometry currentPose;
float currentX, currentY, currentAlpha;
float lookAhead = 2;
float linearSpeed = 0.1;
int nextWayPoint = 1;

float alpha = 0;
float angleSpeed = 0;

void updatePos(const nav_msgs::Odometry::ConstPtr& msg){
  currentPose = *msg;
  currentX = currentPose.pose.pose.position.x;
  currentY = currentPose.pose.pose.position.y;

  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
    
  currentAlpha = yaw;
  //ROS_INFO("Robot position \n\t x = %f \n\t y = %f", currentX, currentY);
}

void computeNextWayPoint(int path[100][2], int pathSize, float lookAhead)
{
  float distance;

  distance = sqrt(pow((currentX - (float)path[nextWayPoint][0]),2) + pow((currentY - (float)path[nextWayPoint][1]),2)) - lookAhead;
  ROS_INFO("Distance to lookAhead: %f", distance);
  if (distance<0.0 && nextWayPoint<pathSize)
  {
    nextWayPoint++;
    ROS_INFO("Moving towards (%d,%d)", path[nextWayPoint][0], path[nextWayPoint][1]);
  }
}

void computeAlpha(int *goal, float lookAhead)
{
  // Hay que transformar las coordenadas globales del WayPoint a los ejes del robot

  // Calcula el vector desde el robot al punto
  float Px, Py, gy;
  Px = goal[0] - currentX;
  Py = goal[1] - currentY;

  // Calcula la proyección del vector del punto 
  // sobre el eje Y del robot

  gy = Px*sin(currentAlpha) + Py*cos(currentAlpha);
  //float alpha = (2*abs(gy)/lookAhead)*angleSpeed;
  alpha = (2*gy/(lookAhead*lookAhead));
  angleSpeed = -alpha;
  //ROS_INFO("Ref = %f \t Err = %f", alpha, err);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  if (argc != 5)
  {
    ROS_INFO("Provide x and y coordinates of goal, look ahead distance (m) and linear speed (m/s)");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient path_client = n.serviceClient<pathtracking::GetPath>("get_path");
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, updatePos);
  pathtracking::GetPath srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  lookAhead = std::stof(argv[3]);
  linearSpeed = std::stof(argv[4]);


  int path_length = 0;
  int path[100][2];

  if (path_client.call(srv))
  {
    ROS_INFO("Path received");
    path_length = sizeof(srv.response.x); // Path size

    for(int i = 0; i < path_length; i++)
    {
      path[i][0] = srv.response.x[i];
      path[i][1] = srv.response.y[i];
      ROS_INFO("(%d,%d)", path[i][0], path[i][1]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service getPath");
    return 1;
  }

  ros::Rate r(SAMPLE_RATE); // 10 hz
  while(ros::ok())
  {
    //ROS_INFO("Length of path = %d", path_length);
    computeNextWayPoint(path, path_length, lookAhead);
    computeAlpha(path[nextWayPoint], lookAhead);

    // Publish the command to change orientation
    geometry_msgs::Twist command;

    command.angular.z = angleSpeed;
    command.angular.x = 0;
    command.angular.y = 0;
    command.linear.x = linearSpeed;
    command.linear.y = 0;
    command.linear.z = 0;

    cmd_pub.publish(command);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}