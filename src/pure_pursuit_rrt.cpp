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
float lookAhead = 1.5;
float linearSpeed = 0.1;
float sat = 1;

int pathSize = 0;
int nextWayPoint = 1;
float goalX, goalY;

float alphaRef = 0;
float angleSpeed = 0;


double rad2deg(double angle){
  double conversion {angle*(180/3.141592)};

  if (conversion > 180) conversion = conversion - 360;
  else if (conversion < -180) conversion = 360 + conversion;
  return conversion;
}

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

void computeNextWayPoint(float path[100][2], int pathSize, float lookAhead)
{
  float lookaheadRelativeDistance;
  lookaheadRelativeDistance = sqrt(pow((currentX - (float)path[nextWayPoint][0]),2) + pow((currentY - (float)path[nextWayPoint][1]),2)) - lookAhead;

  if (lookaheadRelativeDistance<0.0 && nextWayPoint< pathSize-1)
  {
    nextWayPoint++;
  }
}

float computeAlpha(float *goal, float lookAhead, float currentAlpha)
{
  // Hay que transformar las coordenadas globales del WayPoint a los ejes del robot

  // Calcula el vector desde el robot al punto
  float Px, Py, gy, r;
  float alphaError;
  Px = goal[0] - currentX;
  Py = goal[1] - currentY;

  //gy = Py/cos(currentAlpha);
  //r = (lookAhead*lookAhead)/(2*abs(gy));
  //alphaRef = (1/r)*(2*abs(gy))/(lookAhead*lookAhead);
  //alphaError = rad2deg(alphaRef - currentAlpha);

  alphaRef = atan2(Py, Px);
  alphaError = rad2deg(alphaRef - currentAlpha);
  
  return alphaError*(3.141592/180);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  if (argc != 1)
  {
    ROS_INFO("Error calling tracker");
  }


  ros::NodeHandle n;

  n.setParam("rosbot_pure_pursuit_tracker/LookAhead", lookAhead);
  n.setParam("rosbot_pure_pursuit_tracker/LinearVel", linearSpeed);

  ros::ServiceClient path_client = n.serviceClient<pathtracking::GetPath>("get_path");
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, updatePos);
  pathtracking::GetPath srv;

  std::cout << "Type x goal: ";
  std::cin >> goalX;
  std::cout << "Type y goal: ";
  std::cin >> goalY;

  srv.request.x = goalX;
  srv.request.y = goalY;

  n.getParam("rosbot_pure_pursuit_tracker/LookAhead", lookAhead);
  n.getParam("rosbot_pure_pursuit_tracker/LinearVel", linearSpeed);

  float path[100][2];
  

  if (path_client.call(srv))
  {
    pathSize =srv.response.x.size();
    for(int i=0; i<pathSize; i++){
      path[i][0] = srv.response.x[i];
      path[i][1] = srv.response.y[i];
    }
    ROS_INFO("Path received with %d waypoints", pathSize);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  ros::Rate r(SAMPLE_RATE); // 10 hz
  geometry_msgs::Twist command;

  float angleSpeed = 0.0;
  float relativeDistance = 0.0;
  float admissibleDistanceToGoal = 0.25;

  while(ros::ok())
  {
    n.getParam("rosbot_pure_pursuit_tracker/LinearVel", linearSpeed);
    //ROS_INFO("Length of path = %d", pathSize);
    computeNextWayPoint(path, pathSize, lookAhead);
    angleSpeed = computeAlpha(path[nextWayPoint], lookAhead, currentAlpha);

    relativeDistance = sqrt(pow((currentX - path[nextWayPoint][0]),2) + pow((currentY - path[nextWayPoint][1]),2));
    ROS_INFO("Moving to (%f,%f). Relative distance: %.2f. Yaw error: %.2f", path[nextWayPoint][0], path[nextWayPoint][1], relativeDistance, angleSpeed);


    command.angular.x = 0;
    command.angular.y = 0;
    command.angular.z = angleSpeed;
    command.linear.x = linearSpeed;
    command.linear.y = 0;
    command.linear.z = 0;

    if(path[nextWayPoint][0] == (goalX) && path[nextWayPoint][1] == (goalY)){
      if (relativeDistance < admissibleDistanceToGoal) {
        command.angular.z = 0.0;
        command.linear.x = 0.0;
        cmd_pub.publish(command);
        ROS_INFO("Path Completed");
        break;
      }
    }
    else{
      computeNextWayPoint(path, pathSize, lookAhead);
      ROS_INFO("Moving towards (%f,%f)", path[nextWayPoint][0], path[nextWayPoint][1]);
      angleSpeed = computeAlpha(path[nextWayPoint], lookAhead, currentAlpha);
      ROS_INFO("Yaw error: %.2f", angleSpeed);

      command.angular.x = 0;
      command.angular.y = 0;
      command.angular.z = angleSpeed;
      command.linear.x = linearSpeed;
      command.linear.y = 0;
      command.linear.z = 0;
    }

    //saturations

    if(command.angular.z > sat){
      command.angular.z = sat;
    }
    else if (command.angular.z < -sat){
      command.angular.z = -sat;
    }

    cmd_pub.publish(command);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}