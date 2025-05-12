#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <string>

namespace test
{
    class TestNodelet : public nodelet::Nodelet
    {
    public:
    TestNodelet() = default;

    virtual void onInit()
    {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& private_nh = getPrivateNodeHandle();

        std::string yaml_cfg_file;
        if (private_nh.getParam("yaml_cfg_file", yaml_cfg_file)) {
        } else {
        NODELET_WARN("No yaml_cfg_file param specified.");
        }

        NODELET_INFO("TestNodelet initialized.");
    }
    };

} // namespace test

PLUGINLIB_EXPORT_CLASS(test::TestNodelet, nodelet::Nodelet)
