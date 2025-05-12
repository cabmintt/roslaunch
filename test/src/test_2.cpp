#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_2");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("test_2", 10);

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    geometry_msgs::TwistStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.twist.linear.x = 0.2;
    cmd.twist.angular.z = 0.0;

    pub.publish(cmd);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
