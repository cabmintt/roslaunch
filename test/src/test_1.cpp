#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void remap_cb(const geometry_msgs::Twist::ConstPtr& msg);

ros::Publisher pub2;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_1");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("test_1", 10);
  pub2 = nh.advertise<geometry_msgs::Twist>("test_1/remap", 10);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("remap/cmd_vel", 1, remap_cb);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.1;
    cmd.angular.z = 0.0;

    pub.publish(cmd);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void remap_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = msg->linear.x;
    cmd.angular.z = msg->angular.z;

    pub2.publish(cmd);
}
