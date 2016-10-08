/*
 * main.cc
 *
 *  Created on: Sep 12, 2016
 *      Author: root
 */
// ros header file
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/message_filter.h>
// local header file

int main(int argc, char **argv){

  ros::init(argc, argv, "driver");
  int odomPublish_Hz;
  ros::NodeHandle nh;
  sleep(1);

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
	ros::WallTime start = ros::WallTime::now();

    ros::spinOnce();
    loop_rate.sleep();
    ros::WallTime stop = ros::WallTime::now();
  }

  return 0;
}




