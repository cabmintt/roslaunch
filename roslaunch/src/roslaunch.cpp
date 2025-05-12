#include "roslaunch.hpp"

using namespace roslaunch;

LaunchManager::LaunchManager() {
    running_ = true;
}

LaunchManager::~LaunchManager() {
    terminate();
}

LaunchManager& LaunchManager::getLaunchManager() {
    static LaunchManager instance;
    return instance;
}

bool LaunchManager::parse(const std::string &launch_file) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(launch_file.c_str());
    if(!result){
        ROS_ERROR("Failed to load launch file: %s", launch_file.c_str());
        return false;
    }
    pugi::xml_node root = doc.child("launch");
    if(!root){
        ROS_ERROR("No <launch> tag found in file: %s", launch_file.c_str());
        return false;
    }
    for(auto child : root.children()) {
        parseLaunchTag(child, true, "");
    }
    return true;
}


bool LaunchManager::launch() {
    ros::NodeHandle nh;

    for(auto &node : nodes_) {
        replaceArgs(node, arguments_);
    }

    std::string nodelet_manager_name;
    for(auto &node : nodes_) {
        if(node.pkg=="nodelet" && node.args=="manager") {
            nodelet_manager_name = node.name;
            break;
        }
    }

    for(auto &node : nodes_) {
        if(node.pkg=="nodelet" && node.args=="manager") {
            node.currentPid = launchNode(node, true);
            break;
        }
    }
    for(auto &node : nodes_) {
        if(node.pkg=="nodelet" && node.args.find("load")==0) {
            for (auto &p : node.params) {
                std::string param_path;

                if (!node.ns.empty()) {
                    param_path = node.ns;
                    if (param_path.back() != '/') {
                        param_path += "/";
                    }
                }
                else {
                    param_path = "/";
                }

                param_path += node.name;

                if (param_path.back() != '/') {
                    param_path += "/";
                }
                param_path += p.first;
                ros::param::set(param_path, p.second);
            }
            node.currentPid = launchNode(node, false, nodelet_manager_name);
        }
    }
    for(auto &node : nodes_) {
        if(!(node.pkg=="nodelet" &&
            (node.args=="manager" || node.args.find("load")==0))) {
            node.currentPid = launchNode(node, false);
        }
    }

    monitorThread_ = std::thread(&LaunchManager::monitorNodes, this);
    return true;
}

bool LaunchManager::parseLaunchTag(pugi::xml_node xml, bool parentCondition, const std::string& currentNs)
{
    std::string tname = xml.name();
    if(tname == "arg") {
        return parseArg(xml);
    }
    else if(tname == "group") {
        return parseGroup(xml, parentCondition, currentNs);
    }
    else if(tname == "include") {
        if(parentCondition) {
            return parseInclude(xml, parentCondition, currentNs);
        }
        return true;
    }
    else if(tname == "node") {
        if(!parentCondition) return true;
        Node node;
        if(!parseNode(xml, node, currentNs)) return false;
        nodes_.push_back(node);
    }
    return true;
}

bool LaunchManager::parseGroup(pugi::xml_node group_xml, bool parentCondition, const std::string& parentNs) {
    std::string if_expr = group_xml.attribute("if").as_string();
    bool local_cond = parentCondition;
    if(!if_expr.empty()) {
        std::string sub = doSubstitution(if_expr);
        local_cond = evaluateBool(sub) && parentCondition;
    }

    std::string groupNs = group_xml.attribute("ns").as_string();
    groupNs = doSubstitution(groupNs);

    std::string combinedNs = parentNs;
    if(!combinedNs.empty() && !groupNs.empty()) {
        combinedNs += "/" + groupNs;
    } else if(!groupNs.empty()) {
        combinedNs = groupNs;
    }
    for(auto child : group_xml.children()) {
        parseLaunchTag(child, parentCondition, combinedNs);
    }

    return true;
}


bool LaunchManager::parseNode(pugi::xml_node& node_xml, Node& node, const std::string& parentNs) {
    if(!node_xml.attribute("pkg") || !node_xml.attribute("type") || !node_xml.attribute("name")) {
        ROS_ERROR("Node is missing required attributes");
        return false;
    }
    node.pkg  = node_xml.attribute("pkg").as_string();
    node.type = node_xml.attribute("type").as_string();
    node.name = node_xml.attribute("name").as_string();

    if(node_xml.attribute("args")) {
        node.args = node_xml.attribute("args").as_string();
    }
    if(node_xml.attribute("output")) {
        node.output = node_xml.attribute("output").as_string();
    }
    if(node_xml.attribute("respawn")) {
        node.respawn = node_xml.attribute("respawn").as_bool();
    }

    if(node_xml.attribute("ns")) {
        node.ns = doSubstitution(node_xml.attribute("ns").as_string());
    } else {
        node.ns.clear();
    }

    std::string localNs = parentNs;
    // if(node_xml.attribute("ns")) {
    //     std::string subNs = doSubstitution(node_xml.attribute("ns").as_string());
    //     if(!localNs.empty() && !subNs.empty()) {
    //         localNs += "/" + subNs;
    //     } else if(!subNs.empty()) {
    //         localNs = subNs;
    //     }
    // }
    // if(!localNs.empty() && localNs[0] != '/') {
    //     localNs = "/" + localNs;
    // }
    if(!node.ns.empty()) {
        if(!localNs.empty() && localNs.back()=='/' && node.ns.front()=='/')
            localNs += node.ns.substr(1);
        else if(!localNs.empty() && localNs.back()!='/' && node.ns.front()!='/')
            localNs += "/" + node.ns;
        else
            localNs += node.ns;
    }
    node.ns = localNs;

    for(auto child : node_xml.children()) {
        std::string cname = child.name();
        if(cname=="param") {
            std::string pName  = child.attribute("name").as_string();
            std::string pValue = child.attribute("value").as_string();
            if(!pName.empty()) {
                node.params[pName] = pValue;
            }
        }
        else if(cname=="remap") {
            Remap rm;
            rm.from = child.attribute("from").as_string();
            rm.to   = child.attribute("to").as_string();
            node.remaps.push_back(rm);
        }
        else if(cname=="rosparam") {
            Rosparam rp;
            if(parseRosparam(child, rp)) {
                node.rosparams.push_back(rp);
            }
        }
    }
    return true;
}

bool LaunchManager::parseArg(pugi::xml_node arg_xml) {
    std::string arg_name = arg_xml.attribute("name").as_string();
    std::string arg_def  = arg_xml.attribute("default").as_string();
    arg_def = doSubstitution(arg_def);

    auto it = arguments_.find(arg_name);
    if(it == arguments_.end()) {
        arguments_[arg_name] = arg_def;
         if(arg_def.empty()) {
            ROS_INFO("Arg [%s] not set by parent, use default: %s", arg_name.c_str(), arg_def.c_str());
            arguments_[arg_name] = "true";
        } else {
            ROS_INFO("Arg [%s] => [%s] (from default substitution)", arg_name.c_str(), arg_def.c_str());
        }
    } else {
        ROS_INFO("Arg [%s] is already set to [%s], ignoring child default [%s]", arg_name.c_str(), it->second.c_str(), arg_def.c_str());
    }
    return true;
}

bool LaunchManager::parseInclude(pugi::xml_node include_xml, bool condition, const std::string& parentNs) {
    if(!condition) return true;

    std::string fileAttr = include_xml.attribute("file").as_string();
    fileAttr = doSubstitution(fileAttr);

    for(auto argTag : include_xml.children("arg")) {
        std::string argName  = argTag.attribute("name").as_string();
        std::string argValue = argTag.attribute("value").as_string();
        argValue = doSubstitution(argValue);
        arguments_[argName] = argValue;
    }

    pugi::xml_document doc;
    if(!doc.load_file(fileAttr.c_str())) {
        ROS_ERROR("Failed <include> file: %s", fileAttr.c_str());
        return false;
    }
    pugi::xml_node root = doc.child("launch");
    if(!root) {
        ROS_ERROR("No <launch> in included file: %s", fileAttr.c_str());
        return false;
    }

    for(auto child : root.children()) {
        parseLaunchTag(child, true, parentNs);
    }
    return true;
}


bool LaunchManager::parseRosparam(pugi::xml_node& rosparamXml, Rosparam& rpOut) {
    rpOut.command   = rosparamXml.attribute("command").as_string();
    rpOut.file      = doSubstitution(rosparamXml.attribute("file").as_string());
    // rpOut.paramName = rosparamXml.attribute("param").as_string();
    // if(!rosparamXml.text().empty()) {
    //     rpOut.text = rosparamXml.text().as_string();
    // }
    return true;
}



std::string LaunchManager::doSubstitution(const std::string& in) {
    std::string out = in;
    bool replaced=false;
    do {
        replaced=false;
        size_t envStart= out.find("$(env ");
        size_t optenvStart= out.find("$(optenv ");
        size_t findStart= out.find("$(find ");
        size_t evalStart= out.find("$(eval ");

        if(envStart!=std::string::npos &&
           (optenvStart==std::string::npos||envStart<optenvStart) &&
           (findStart==std::string::npos  ||envStart<findStart) &&
           (evalStart==std::string::npos  ||envStart<evalStart)) {
            size_t end= out.find(")", envStart);
            if(end!=std::string::npos) {
                std::string var= out.substr(envStart+6, end-(envStart+6));
                const char* v= getenv(var.c_str());
                std::string val= (v? v:"");
                out.replace(envStart, end-envStart+1, val);
                replaced=true;
            }
        }
        else if(optenvStart!=std::string::npos &&
                (findStart==std::string::npos||optenvStart<findStart) &&
                (evalStart==std::string::npos||optenvStart<evalStart)) {
            size_t end= out.find(")", optenvStart);
            if(end!=std::string::npos) {
                std::string varDef= out.substr(optenvStart+9, end-(optenvStart+9));
                std::istringstream iss(varDef);
                std::string var, def;
                iss>>var>>def;
                const char* v= getenv(var.c_str());
                std::string val= (v? v:def);
                out.replace(optenvStart, end-optenvStart+1, val);
                replaced=true;
            }
        }
        else if(findStart!=std::string::npos &&
                (evalStart==std::string::npos||findStart<evalStart)) {
            size_t end= out.find(")", findStart);
            if(end!=std::string::npos) {
                std::string pkg= out.substr(findStart+7, end-(findStart+7));
                std::string path= ros::package::getPath(pkg);
                out.replace(findStart, end-findStart+1, path);
                replaced=true;
            }
        }
        else if(evalStart!=std::string::npos) {
            size_t end= out.find(")", evalStart);
            if(end!=std::string::npos) {
                std::string expr= out.substr(evalStart+7, end-(evalStart+7));
                std::string result= evalCondition(expr);
                out.replace(evalStart, end-evalStart+1, result);
                replaced=true;
            }
        }
    } while(replaced);
    return out;
}

std::string LaunchManager::evalCondition(const std::string& expr) {
    size_t eqpos= expr.find("==");
    if(eqpos==std::string::npos) {
        return expr;
    }
    std::string lhs= expr.substr(0, eqpos);
    std::string rhs= expr.substr(eqpos+2);

    auto trim=[&](std::string&s){
        while(!s.empty()&&isspace(s.front())) s.erase(s.begin());
        while(!s.empty()&&isspace(s.back())) s.pop_back();
    };
    trim(lhs); trim(rhs);

    if(!rhs.empty()&&rhs.front()=='\'') rhs.erase(rhs.begin());
    if(!rhs.empty()&&rhs.back()=='\'') rhs.pop_back();

    auto it= arguments_.find(lhs);
    std::string lhsVal= (it==arguments_.end())? "" : it->second;
    return (lhsVal==rhs)? "true":"false";
}

bool LaunchManager::evaluateBool(const std::string& expr) {
    std::string e= expr;
    for(auto &c: e) c= tolower(c);
    if(e=="true"|| e=="1") return true;
    return false;
}



void LaunchManager::replaceArg(std::string& factor, const std::map<std::string, std::string> &arg_map) {
    bool replaced=false;
    do {
        replaced=false;
        size_t arg_start= factor.find("$(arg ");
        size_t find_start= factor.find("$(find ");
        if(arg_start!=std::string::npos &&
           (find_start==std::string::npos||arg_start<find_start)) {
            size_t end= factor.find(")", arg_start);
            if(end!=std::string::npos) {
                std::string key= factor.substr(arg_start+6, end-(arg_start+6));
                auto it= arg_map.find(key);
                if(it!=arg_map.end()) {
                    factor.replace(arg_start, end-arg_start+1, it->second);
                } else {
                    factor.erase(arg_start, end-arg_start+1);
                }
                replaced=true;
            }
        }
        else if(find_start!=std::string::npos) {
            size_t end= factor.find(")", find_start);
            if(end!=std::string::npos) {
                std::string pkg= factor.substr(find_start+7, end-(find_start+7));
                std::string pkg_path= ros::package::getPath(pkg);
                if(!pkg_path.empty()) {
                    factor.replace(find_start, end-find_start+1, pkg_path);
                } else {
                    factor.erase(find_start, end-find_start+1);
                }
                replaced=true;
            }
        }
    } while(replaced);
}

void LaunchManager::replaceArgs(Node& node, const std::map<std::string, std::string>& arg_map) {
    replaceArg(node.pkg, arg_map);
    replaceArg(node.type, arg_map);
    replaceArg(node.name, arg_map);
    replaceArg(node.args, arg_map);
    replaceArg(node.output, arg_map);
    replaceArg(node.ns, arg_map);
    for(auto &p: node.params) {
        replaceArg(p.second, arg_map);
    }
    for(auto &rm: node.remaps) {
        replaceArg(rm.from, arg_map);
        replaceArg(rm.to, arg_map);
    }
    for(auto &rp: node.rosparams) {
        rp.file= doSubstitution(rp.file);
        rp.paramName= doSubstitution(rp.paramName);
    }
}

std::vector<std::string> LaunchManager::splitString(const std::string& str) {
    std::istringstream iss(str);
    std::vector<std::string> tokens;
    std::string token;
    while(iss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

std::string LaunchManager::findExecutable(const std::string& pkg, const std::string& type) {
    std::string pkg_path = ros::package::getPath(pkg);
    if(pkg_path.empty()) {
        ROS_ERROR("Package [%s] not found", pkg.c_str());
        return "";
    }
    std::vector<std::string> paths;
    if(pkg_path.find("/opt/ros/") != std::string::npos) {
        size_t pos = pkg_path.find("/share/");
        if(pos != std::string::npos) {
            std::string prefix = pkg_path.substr(0,pos);
            paths.push_back(prefix + "/lib/" + pkg + "/" + type);
            paths.push_back(prefix + "/lib/" + type);
        }
    }
    paths.push_back("/home/syscon/catkin_ws/devel/lib/"+pkg+"/"+type);
    paths.push_back("/home/syscon/catkin_ws/install/lib/"+pkg+"/"+type);
    paths.push_back("/home/syscon/catkin_ws/bin/"+type);

    for(auto &p: paths) {
        if(access(p.c_str(), X_OK)==0) {
            return p;
        }
    }
    ROS_ERROR("Executable [%s] not found in pkg [%s]", type.c_str(), pkg.c_str());
    return "";
}


pid_t LaunchManager::launchNode(Node& node, bool is_manager, const std::string& manager_name) {
    loadRosparams(node.rosparams, node.name, node.ns);

    pid_t pid = fork();
    if(pid>0) {
        childPids_.push_back(pid);
        ROS_INFO("Launched node [%s] PID=%d respawn=%d ns=%s", node.name.c_str(), pid, (int)node.respawn, node.ns.c_str());
        return pid;
    } else if(pid==0) {
        for(auto &p: node.params) {
            ros::param::set(p.first, p.second);
        }

        std::vector<std::string> exec_args;
        if(node.pkg=="nodelet" && is_manager) {
            exec_args.push_back("manager");
            exec_args.push_back(std::string("__name:=")+ node.name);
            if(!node.ns.empty()) {
                if(node.ns[0] != '/') node.ns = "/" + node.ns;
                exec_args.push_back(std::string("__ns:=")+ node.ns);
            }
        }
        else if(node.pkg=="nodelet" && node.args.find("load")==0) {
            std::vector<std::string> tokens = splitString(node.args);
            exec_args.insert(exec_args.end(), tokens.begin(), tokens.end());
            exec_args.push_back("__name:=" + node.name);
            if(!node.ns.empty()) {
                if(node.ns[0] != '/') node.ns = "/" + node.ns;
                exec_args.push_back("__ns:=" + node.ns);
            }
        }
        else {
            if(!node.args.empty()) {
                std::vector<std::string> tokens = splitString(node.args);
                exec_args.insert(exec_args.end(), tokens.begin(), tokens.end());
            }
            exec_args.push_back("__name:=" + node.name);
            if(!node.ns.empty()) {
                if(node.ns[0] != '/') node.ns = "/" + node.ns;
                exec_args.push_back("__ns:=" + node.ns);
            }
        }
        for(auto &rm: node.remaps) {
            exec_args.push_back(rm.from + ":=" + rm.to);
        }

        std::string exec_path = findExecutable(node.pkg, node.type);
        if(exec_path.empty()) {
            ROS_ERROR("Executable not found for [%s]", node.name.c_str());
            exit(EXIT_FAILURE);
        }
        std::vector<char*> args;
        args.push_back(const_cast<char*>(exec_path.c_str()));
        for(auto &arg_str : exec_args) {
            args.push_back(const_cast<char*>(arg_str.c_str()));
        }
        args.push_back(nullptr);

        execvp(exec_path.c_str(), args.data());
        ROS_ERROR("Failed exec node path [%s]", exec_path.c_str());
        exit(EXIT_FAILURE);
    } else {
        ROS_ERROR("Failed to fork for node %s", node.name.c_str());
        return false;
    }
}


void LaunchManager::loadRosparams(const std::vector<Rosparam>& rosparams, const std::string& nodeName, const std::string& ns) {
    std::string prefix;
    if(!ns.empty()) {
        if(ns[0] != '/') prefix = "/" + ns;
        else prefix = ns;
    }
    if(!prefix.empty()) prefix += "/" + nodeName;
    else prefix = "/" + nodeName;

    for(const auto &rp: rosparams) {
        if(rp.command=="load") {
            if(!rp.file.empty()) {
                try {
                    YAML::Node root = YAML::LoadFile(rp.file);
                    mergeYamlToRosparams(root, prefix);
                    ROS_INFO("rosparam load file=%s => %s", rp.file.c_str(), prefix.c_str());
                }
                catch(std::exception &e) {
                    ROS_ERROR("Failed to load YAML file: %s, err=%s", rp.file.c_str(), e.what());
                }
            }
        }
    }
}


void LaunchManager::mergeYamlToRosparamString(const YAML::Node &node, const std::string &key)
{
    if (node.IsSequence()) {
        YAML::Emitter emitter;
        emitter << node;
        if (emitter.good()) {
            std::string yamlString = emitter.c_str();
            ros::param::set(key, yamlString);
            ROS_ERROR("Failed to emit YAML sequence for key=[%s]", key.c_str());
        }
    }
    else if (node.IsMap()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            std::string childKey = it->first.as<std::string>();
            std::string fullKey  = key;
            if (!fullKey.empty() && fullKey.back() != '/')
                fullKey += "/";
            fullKey += childKey;
            mergeYamlToRosparams(it->second, fullKey);
        }
    }
    else if (node.IsScalar()) {
        std::string scalar = node.as<std::string>();
        try {
            int val_i = node.as<int>();
            ros::param::set(key, val_i);
            return;
        } catch(...) {}
        try {
            double val_d = node.as<double>();
            ros::param::set(key, val_d);
            return;
        } catch(...) {}
        if (scalar == "true" || scalar == "True") {
            ros::param::set(key, true);
        } else if (scalar == "false" || scalar == "False") {
            ros::param::set(key, false);
        } else {
            ros::param::set(key, scalar);
        }
        ROS_INFO("Set param [%s] as scalar", key.c_str());
    }
    else {
        ros::param::set(key, "");
        ROS_INFO("Set param [%s] as empty string", key.c_str());
    }
}


void LaunchManager::mergeYamlToRosparamsEach(const YAML::Node& node, const std::string& key) {
    if (node.IsMap()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            std::string childKey = it->first.as<std::string>();
            std::string fullKey  = key;
            if (!fullKey.empty() && fullKey.back() != '/') {
                fullKey += "/";
            }
            fullKey += childKey;

            mergeYamlToRosparams(it->second, fullKey);
        }
    }
    else if (node.IsSequence()) {
        for (size_t i = 0; i < node.size(); i++) {
            std::string idxKey = key;
            if (!idxKey.empty() && idxKey.back() != '/') {
                idxKey += "/";
            }
            idxKey += std::to_string(i);
            mergeYamlToRosparams(node[i], idxKey);
        }
    }
    else if (node.IsScalar()) {
        std::string scalar = node.as<std::string>();
        try {
            int val_i = node.as<int>();
            ros::param::set(key, val_i);
            return;
        } catch(...) {}
        try {
            double val_d = node.as<double>();
            ros::param::set(key, val_d);
            return;
        } catch(...) {}
        if (scalar == "true" || scalar == "True") {
            ros::param::set(key, true);
        } else if (scalar == "false" || scalar == "False") {
            ros::param::set(key, false);
        } else {
            ros::param::set(key, scalar);
        }
    }
    else {
        ros::param::set(key, "");
    }
}


void LaunchManager::mergeYamlToRosparams(const YAML::Node &node, const std::string &key)
{
    try {
        XmlRpc::XmlRpcValue val = yamlToXmlRpc(node);
        if (!val.valid()) {
            ROS_WARN("XmlRpcValue invalid for key=[%s]", key.c_str());
            return;
        }
        ros::param::set(key, val);
        ROS_INFO("Set param [%s] as array/map via xmlrpc", key.c_str());
    }
    catch (const XmlRpc::XmlRpcException &xe) {
        ROS_ERROR("XmlRpc error while converting [%s]: %s", 
                  key.c_str(), xe.getMessage().c_str());
    }
    catch (const std::exception &e) {
        ROS_ERROR("Std exception while converting [%s]: %s", 
                  key.c_str(), e.what());
    }
}

XmlRpc::XmlRpcValue LaunchManager::yamlToXmlRpc(const YAML::Node &node)
{
    XmlRpc::XmlRpcValue v;
    if (node.IsMap()) {
        v.setSize(0); // struct
        for (auto it = node.begin(); it != node.end(); ++it) {
            std::string key = it->first.as<std::string>();
            XmlRpc::XmlRpcValue child = yamlToXmlRpc(it->second);
            v[key] = child; 
        }
    }
    else if (node.IsSequence()) {
        v.setSize(node.size()); // array
        for (size_t i = 0; i < node.size(); i++) {
            XmlRpc::XmlRpcValue child = yamlToXmlRpc(node[i]);
            v[i] = child; 
        }
    }
    else if (node.IsScalar()) {
        std::string scalar = node.as<std::string>();
        try {
            int vi = node.as<int>();
            v = vi;
            return v;
        } catch(...) {}
        try {
            double vd = node.as<double>();
            v = vd;
            return v;
        } catch(...) {}
        if (scalar == "true" || scalar == "True") {
            v = true;
        } else if (scalar == "false" || scalar == "False") {
            v = false;
        } else {
            v = scalar;
        }
    }
    else {
        v = "";
    }
    return v;
}




void LaunchManager::monitorNodes() {
    while(running_) {
        for(auto &node: nodes_) {
            if(node.currentPid <=0) continue;
            int status;
            pid_t res = waitpid(node.currentPid, &status, WNOHANG);
            if(res==0) {
            } else if(res==-1) {
                ROS_ERROR("Error while waiting for PID %d", node.currentPid);
                node.currentPid=-1;
            } else {
                pid_t oldPid = node.currentPid;
                auto it = std::find(childPids_.begin(), childPids_.end(), oldPid);
                if(it != childPids_.end()) {
                    childPids_.erase(it);
                }
                node.currentPid=-1;
                if(WIFSIGNALED(status)) {
                    ROS_ERROR("Node [%s] pid=%d signaled %d",
                              node.name.c_str(), oldPid, WTERMSIG(status));
                } else if(WIFEXITED(status)) {
                    ROS_INFO("Node [%s] pid=%d exit code=%d",
                             node.name.c_str(), oldPid, WEXITSTATUS(status));
                } else {
                    ROS_INFO("Node [%s] pid=%d terminated.",
                             node.name.c_str(), oldPid);
                }
                if(node.respawn && running_) {
                    ROS_WARN("Respawning node [%s]...", node.name.c_str());
                    node.currentPid = launchNode(node, false);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void LaunchManager::terminate() {
    if(!running_) return;
    running_=false;
    if(monitorThread_.joinable()) {
        monitorThread_.join();
    }
    for(auto pid: childPids_) {
        if(kill(pid, SIGINT)==0) {
            ROS_INFO("Sent SIGINT to PID %d", pid);
        } else {
            ROS_WARN("Failed SIGINT PID %d", pid);
        }
    }
    for(auto pid : childPids_) {
        int status;
        pid_t result = waitpid(pid, &status, 0);
        if(result == -1) {
            ROS_ERROR("Error while waiting for PID %d", pid);
        } else {
            if(WIFSIGNALED(status)) {
                ROS_ERROR("Node with PID %d exited due to signal %d.", pid, WTERMSIG(status));
            } else if(WIFEXITED(status)) {
                ROS_INFO("Node with PID %d exited with status %d.", pid, WEXITSTATUS(status));
            }
            else {
                ROS_INFO("Node with PID %d terminated.", pid);
            }
        }
    }

    childPids_.clear();
    ros::shutdown();
    std::cout << "[ INFO] All child nodes terminated" << std::endl;
}
