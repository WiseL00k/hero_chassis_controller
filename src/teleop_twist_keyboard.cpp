//
// Created by wiselook on 2024/12/12.
//

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv)
{
  // teleop_twist_keyboard node init.
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;

  // cmd_vel publisher init
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  geometry_msgs::Twist twist;

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}