#include "roslaunch.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "roslaunch");
    ros::NodeHandle nh;

    std::string file = "/home/syscon/catkin_ws/src/test_repo/test/launch/test_1.launch";

    if(!roslaunch::parseLaunch(file)) {
        ROS_ERROR("Failed to parse launch file.");
        return false;
    }
    if(!roslaunch::startLaunch()) {
        ROS_ERROR("Failed to launch nodes.");
        return false;
    }

    ros::spin();
    return 0;
}


