#define SAMPLE_RATE 10
<<<<<<< HEAD
=======
#define SPEED 1
>>>>>>> 41df8229ed232a0727df5360d024b714c9bd169a

#include "ros/ros.h"
#include "math.h"
#include "pathtracking/GetPath.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cstdlib>

<<<<<<< HEAD
#include <tf/tf.h>


nav_msgs::Odometry currentPose;
float currentX, currentY, currentAlpha;
float lookAhead = 2;
float linearSpeed = 0.1;
int nextWayPoint = 1;

float alpha = 0;
float angleSpeed = 0;
=======

nav_msgs::Odometry currentPose;
float currentX, currentY;
float lookAhead = 1;
>>>>>>> 41df8229ed232a0727df5360d024b714c9bd169a

void updatePos(const nav_msgs::Odometry::ConstPtr& msg){
  currentPose = *msg;
  currentX = currentPose.pose.pose.position.x;
  currentY = currentPose.pose.pose.position.y;
<<<<<<< HEAD

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
=======
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
>>>>>>> 41df8229ed232a0727df5360d024b714c9bd169a
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  if (argc != 5)
  {
<<<<<<< HEAD
    ROS_INFO("Provide x and y coordinates of goal, look ahead distance (m) and linear speed (m/s)");
=======
    ROS_INFO("Provide x and y coordinates of goal");
>>>>>>> 41df8229ed232a0727df5360d024b714c9bd169a
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient path_client = n.serviceClient<pathtracking::GetPath>("get_path");
<<<<<<< HEAD
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, updatePos);
  pathtracking::GetPath srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  lookAhead = std::stof(argv[3]);
  linearSpeed = std::stof(argv[4]);


  int path_length = 0;
  int path[100][2];
=======
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, updatePos);
  pathtracking::GetPath srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);

  int pathSize;
  float** path;
>>>>>>> 41df8229ed232a0727df5360d024b714c9bd169a

  if (path_client.call(srv))
  {
    ROS_INFO("Path received");
<<<<<<< HEAD
    path_length = sizeof(srv.response.x); // Path size

    for(int i = 0; i < path_length; i++)
    {
      path[i][0] = srv.response.x[i];
      path[i][1] = srv.response.y[i];
      ROS_INFO("(%d,%d)", path[i][0], path[i][1]);
=======
    int pathSize = srv.response.x.size(); // Path size
    float path[pathSize][2];

    for(int i = 0; i < pathSize; i++)
    {
      path[i][0] = srv.response.x[i];
      path[i][1] = srv.response.y[i];
>>>>>>> 41df8229ed232a0727df5360d024b714c9bd169a
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
<<<<<<< HEAD
    //ROS_INFO("Length of path = %d", path_length);
    computeNextWayPoint(path, path_length, lookAhead);
    computeAlpha(path[nextWayPoint], lookAhead);
=======
    int closest = computeNearestWayPoint(path, pathSize, lookAhead);
    float alpha = computeAlpha(path[closest], lookAhead);
>>>>>>> 41df8229ed232a0727df5360d024b714c9bd169a

    // Publish the command to change orientation
    geometry_msgs::Twist command;

<<<<<<< HEAD
    command.angular.z = angleSpeed;
    command.angular.x = 0;
    command.angular.y = 0;
    command.linear.x = linearSpeed;
=======
    command.angular.z = -alpha;
    command.angular.x = 0;
    command.angular.y = 0;
    command.linear.x = SPEED;
>>>>>>> 41df8229ed232a0727df5360d024b714c9bd169a
    command.linear.y = 0;
    command.linear.z = 0;

    cmd_pub.publish(command);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}