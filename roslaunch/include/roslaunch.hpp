#pragma once

#include <string>
#include <vector>
#include <map>
#include <csignal>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <pugixml.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <signal.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>
#include <cstdlib>
#include <yaml-cpp/yaml.h>
#include <ros/param.h>
#include <XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <ros/xmlrpc_manager.h>

#define ARG_LEN 6
#define FIND_LEN 7

namespace roslaunch {
    struct Remap {
        std::string from;
        std::string to;
    };

    struct Rosparam {
        std::string command;
        std::string file;
        std::string paramName;
        std::string text;
    };

    struct Node {
        std::string pkg;
        std::string type;
        std::string name;
        std::string args;
        std::string output;
        // std::string respawn;
        bool respawn = false;
        bool if_condition = true;
        std::string ns;
        std::map<std::string, std::string> params;
        // std::map<std::string, std::string> remaps;
        std::vector<Remap> remaps;
        std::vector<Rosparam> rosparams;
        pid_t currentPid = -1;
    };

    class LaunchManager {
        public:
            LaunchManager();
            ~LaunchManager();

            static LaunchManager& getLaunchManager();

            bool parse(const std::string &file);
            bool launch();

        private:
            bool parseLaunchTag(pugi::xml_node xml, bool parentCondition, const std::string& currentNs);
            bool parseGroup(pugi::xml_node group_xml, bool parentCondition, const std::string& parentNs);
            bool parseNode(pugi::xml_node& node_xml, Node& node, const std::string& parentNs);
            bool parseArg(pugi::xml_node arg_xml);
            bool parseInclude(pugi::xml_node include_xml, bool condition, const std::string& parentNs);
            bool parseRosparam(pugi::xml_node& rosparamXml, Rosparam& rpOut);

            std::string doSubstitution(const std::string& in);
            std::string evalCondition(const std::string& expr);
            bool evaluateBool(const std::string& expr);

            void replaceArg(std::string& factor, const std::map<std::string, std::string> &arg_map);
            void replaceArgs(Node& node, const std::map<std::string, std::string>& arg_map);

            std::vector<std::string> splitString(const std::string& str);
            std::string findExecutable(const std::string& pkg, const std::string& type);

            pid_t launchNode(Node& node, bool is_manager, const std::string& manager_name = "");
            void loadRosparams(const std::vector<Rosparam>& rosparams, const std::string& nodeName, const std::string& ns);
            void mergeYamlToRosparamString(const YAML::Node& node, const std::string& key);
            void mergeYamlToRosparamsEach(const YAML::Node& node, const std::string& key);
            void mergeYamlToRosparams(const YAML::Node &node, const std::string &key);
            static XmlRpc::XmlRpcValue yamlToXmlRpc(const YAML::Node& node);

            void monitorNodes();
            void terminate();

            std::map<std::string, std::string> arguments_;
            std::vector<Node> nodes_;
            std::vector<pid_t> childPids_;
            std::thread monitorThread_;
            bool running_;

    };  // class

    inline bool parseLaunch(const std::string& file) {
        auto& mgr = LaunchManager::getLaunchManager();
        return mgr.parse(file);
    }

    inline bool startLaunch() {
        auto& mgr = LaunchManager::getLaunchManager();
        return mgr.launch();
    }

} // namespace
