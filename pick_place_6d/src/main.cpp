#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <pose_detection/BroadcastPoseAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <chrono>
#include <thread>

#include <sstream>

#include "picking.hpp"
#include "ros/ros.h"
constexpr static int TOOL_ID(13);

template <typename... T>
void readParameters(const ros::NodeHandle &nh, T &... args) {
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
    ROS_WARN_STREAM("Starting the node");
    ros::init(argc, argv, "new_pick_place");
    ros::NodeHandle nh;
    std::pair<std::string, std::string> _topic("/new_pick_place_exe/grasp_pose", "");
    readParameters(nh, _topic);
    Picking picker;
    picker.connectToRobot(nh);
    ROS_WARN_STREAM("Connected to robot");
    ROS_WARN_STREAM("waiting for topic" << _topic.second);
    nh.subscribe(_topic.second, 1, &Picking::callback, &picker);
    ros::spin();
    return 0;
}
