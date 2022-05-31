#include "ros/ros.h"
#include "pathtracking/GetPath.h"
#include <cstdlib>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>

/*Callback de odometria*/
nav_msgs::Odometry odom;
geometry_msgs::Pose2D pose2d;
geometry_msgs::Pose2D error;
geometry_msgs::Pose2D prev_error;

geometry_msgs::Pose2D ref;


void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw;
}

bool isOriented(float error_angle, float threshold){
  return abs(error_angle) <= threshold;
}

bool reachedPoint(float error, float threshold){
  return error <= threshold;
}

double rad2deg(double angle){
  double conversion {angle*(180/3.141592)};

  if (conversion > 180) conversion = conversion - 360;
  else if (conversion < -180) conversion = 360 + conversion;
  return conversion;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  if (argc != 1)
  {
    ROS_INFO("usage: basic pid control algorithm");
  }

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<pathtracking::GetPath>("get_path");

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>
            ("/cmd_vel", 10);

  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry> /*Odometry*/
            ("/odom", 10, odom_cb);

  geometry_msgs::Twist twist;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  int control_freq = 10; 
  double tm = 1/control_freq;
  ros::Rate loop_rate(control_freq);
  
  pathtracking::GetPath srv;

  double P_linear = 0.1;
  double I_linear = 0.01;

  double P_angular = 0.1;
  double I_angular = 0.01;

  n.setParam("rosbot_pid_tracker/P_linear", P_linear);
  n.setParam("rosbot_pid_tracker/I_linear", I_linear);

  n.setParam("rosbot_pid_tracker/P_angular", P_angular);
  n.setParam("rosbot_pid_tracker/I_angular", I_angular);

  double sat = 1.0;

  double sqrt_error = 0.0;

  double orientation_angle_threshold = 5; //degrees
  double distance_threshold = 0.15; //meters


  double linear_speed = 0.2;
  double angular_speed = 0.1;

  float path[100][2];
  int path_length = 0;
  double goal_x, goal_y;

  std::cout << "Type x goal: ";
  std::cin >> goal_x;
  std::cout << "Type y goal: ";
  std::cin >> goal_y;

  /*int goal_x = atoll(argv[1]);
  int goal_y = atoll(argv[2]);*/

  srv.request.x = goal_x;
  srv.request.y = goal_y;

  if (client.call(srv))
  {
    path_length = srv.response.x.size();
    for(int i=0; i<path_length; i++){
      path[i][0] = srv.response.x[i] + 0.5;
      path[i][1] = srv.response.y[i] + 0.5;
    }
    ROS_INFO("Path received with %d waypoints", path_length);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  int i = 1;
  bool oriented = 0;

  while(ros::ok()){

    ref.x = path[i][0];
    ref.y = path[i][1];

    error.x = ref.x - pose2d.x;
    error.y = ref.y - pose2d.y;

    sqrt_error = sqrt(pow(error.x, 2) + pow(error.y,2));

    ref.theta = atan2(error.y, error.x);

    error.theta = rad2deg(ref.theta - pose2d.theta);

    n.getParam("rosbot_pid_tracker/P_linear", P_linear);
    n.getParam("rosbot_pid_tracker/I_linear", I_linear);

    n.getParam("rosbot_pid_tracker/P_angular", P_angular);
    n.getParam("rosbot_pid_tracker/I_angular", I_angular);



    if(reachedPoint(sqrt_error, distance_threshold)){  
      ROS_INFO("Waypoint %d: (%f,%f) reached", i, path[i][0], path[i][1]); 
      
      if (path[i][0] == goal_x + 0.5 && path[i][1] == goal_y + 0.5){
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = 0.0;
        vel_pub.publish(twist); /*Para parar el robot*/
        ROS_INFO("Path completed");
        break;
      } 
      i++;
      oriented = 0;
    }

    else if (oriented){
      twist.linear.x = P_linear*sqrt_error + I_linear*(error.x - prev_error.x)*tm;
      twist.angular.z = P_angular*error.theta + I_angular*(error.theta - prev_error.theta)*tm;

      /*if(error.theta > 0.0) twist.angular.z = P_angular*error.theta + I_angular*(error.theta - prev_error.theta)*tm;
      else twist.angular.z = - (P_angular*error.theta + I_angular*(error.theta - prev_error.theta)*tm);*/
      }

    else{
      twist.linear.x = 0.0;
      twist.angular.z = P_angular*error.theta + I_angular*(error.theta - prev_error.theta)*tm;
       ROS_INFO("Orientating towards first goal");
      if (isOriented(error.theta, orientation_angle_threshold)){
        oriented = 1;
      }
    }

    ROS_INFO("Moving to wp %d: (%f,%f). Distance =  %.2f. Yaw error= %.2f degrees", i, path[i][0], path[i][1], sqrt_error, error.theta);

    // Saturaciones
    if (twist.linear.x >= sat) twist.linear.x = sat;
    else if (twist.linear.x <= -sat) twist.linear.x = -sat;

    if (twist.angular.z >= sat) twist.angular.z = sat;
    else if (twist.angular.z <= -sat) twist.angular.z = -sat;

    vel_pub.publish(twist);
    prev_error = error;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
