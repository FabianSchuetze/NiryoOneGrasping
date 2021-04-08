#include "integrate.hpp"
#include <filesystem>
#include <ros/ros.h>

using namespace integration;

template <typename... T> void fold(const ros::NodeHandle &nh, T &... args) {
    auto read_parameters = [&](auto &t) {
        nh.getParam(t.first, t.second);
        if (t.second.empty()) {
            ROS_WARN_STREAM("Rosparam " << t.first << " not identified");
            throw std::runtime_error("Could not read all parameters");
        }
        ROS_WARN_STREAM("The parameters for " << t.first << " is " << t.second);
    };
    (..., read_parameters(args));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    std::pair<std::string, std::string> root("integrate/root", "");
    fold(nh, root);
    Integration integrate(root.second);
    auto scene = integrate.integrate();
    o3d::visualization::DrawGeometries({scene}, "Cluster");
}
