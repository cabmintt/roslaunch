#include <ros/ros.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <geometry_msgs/TwistWithCovariance.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_3");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::TwistWithCovariance>("test_3", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::TwistWithCovariance cmd;
    cmd.twist.linear.x = 0.3;
    cmd.twist.angular.z = 0.0;

    pub.publish(cmd);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

