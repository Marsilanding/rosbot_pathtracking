#define SAMPLE_RATE 10

#include "ros/ros.h"
#include "math.h"
#include "pathtracking/GetPath.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cstdlib>

#include <tf/tf.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

nav_msgs::Odometry currentPose;
float currentX, currentY, currentAlpha;
float lookAhead = 1.5;
float linearSpeed = 0.1;
float sat = 1;
int nextWayPoint = 1;
int goalX, goalY;

float alphaRef = 0;
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
  float lookaheadRelativeDistance;
  lookaheadRelativeDistance = sqrt(pow((currentX - (float)path[nextWayPoint][0]),2) + pow((currentY - (float)path[nextWayPoint][1]),2)) - lookAhead;

  if (lookaheadRelativeDistance<0.0 && nextWayPoint< pathSize-1)
  {
    nextWayPoint++;
  }
}

float computeAlpha(geometry_msgs::PoseStamped &wp, float lookAhead, float currentAlpha)
{
  // Hay que transformar las coordenadas globales del WayPoint a los ejes del robot

  // Calcula el vector desde el robot al punto
  int Px, Py;
  float gy, r;
  float thetaWorld, relativeTheta;
  float alphaError;

  Px = wp.pose.position.x;
  Py = wp.pose.position.y;

  gy = Py;

  r = (lookAhead*lookAhead)/(2*abs(gy));

  alphaRef = (1/r)*(2*abs(gy))/(lookAhead*lookAhead);

  ROS_INFO("gy = %.2f, r = %.2f, alphaRef = %.2f", gy, r, alphaRef);

  alphaError = alphaRef - currentAlpha;

  if (alphaError > 180) alphaError-=360;
  else if (alphaError < -180) alphaError+= 360;
  
  return alphaRef;
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
  std::cout << "Type x goal: ";
  std::cin >> goalY;

  srv.request.x = goalX;
  srv.request.y = goalY;

  n.getParam("rosbot_pure_pursuit_tracker/LookAhead", lookAhead);
  n.getParam("rosbot_pure_pursuit_tracker/LinearVel", linearSpeed);

  int pathSize = 0;
  int path[100][2];
  

  if (path_client.call(srv))
  {
    pathSize = sizeof(srv.response.x);
    for(int i=0; i<100; i++){
      path[i][0] = srv.response.x[i];
      path[i][1] = srv.response.y[i];
      if (path[i][0] == goalX && path[i][1] == goalY){
        pathSize = i+1;
        break;
      }
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
  geometry_msgs::PoseStamped wp;
  geometry_msgs::TransformStamped odom_to_base_link;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);


  float angleSpeed = 0.0;
  float relativeDistance = 0.0;
  float admissibleDistanceToGoal = 0.3;

  while(ros::ok())
  {

  //ROS_INFO("Length of path = %d", pathSize);

    computeNextWayPoint(path, pathSize, lookAhead);
    wp.pose.position.x = path[nextWayPoint][0];
    wp.pose.position.y = path[nextWayPoint][1];
    wp.header.frame_id = "odom";
    angleSpeed = computeAlpha(wp, lookAhead, currentAlpha);
    ROS_INFO("Moving to (%d,%d). Relative distance: %.2f. Yaw reference: %.2f", path[nextWayPoint][0], path[nextWayPoint][1], relativeDistance, angleSpeed);

    wp.header.frame_id = "odom";

    odom_to_base_link = tf_buffer.lookupTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0) );
    tf2::doTransform(wp, wp, odom_to_base_link); // robot_pose is the PoseStamped I want to transform

    relativeDistance = sqrt(pow(wp.pose.position.x,2) + pow(wp.pose.position.y,2));
    ROS_INFO("Moving to (%d,%d). Relative distance: %.2f. Yaw error: %.2f", path[nextWayPoint][0], path[nextWayPoint][1], relativeDistance, angleSpeed);


    command.angular.x = 0;
    command.angular.y = 0;
    command.angular.z = angleSpeed;
    command.linear.x = linearSpeed;
    command.linear.y = 0;
    command.linear.z = 0;

    if(path[nextWayPoint][0] == goalX && path[nextWayPoint][1] == goalY){
      if (relativeDistance < admissibleDistanceToGoal) {
        command.angular.z = 0.0;
        command.linear.x = 0.0;
        ROS_INFO("Path Completed");
      }
    }
    else{

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