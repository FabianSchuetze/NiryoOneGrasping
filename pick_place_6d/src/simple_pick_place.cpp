#include "picking.hpp"
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <sstream>

#include "ros/ros.h"
static constexpr double SAFTY_Z(0.1);

Picking::EEFPosition computePosition(const std::vector<double> &goal,
                                     bool open_gripper) {
    double z = goal[2];
    if (z < SAFTY_Z) {
        z = SAFTY_Z;
    }
    auto pose = Picking::Position(goal[0], goal[1], z, goal[3], goal[4],
                                  goal[5], open_gripper);
    return pose;
}

std::vector<double> parseInput(const std::string &input) {
    const std::string tmp =
        "Enter" + input + " elements, separated by whitespace";
    ROS_INFO("%s", tmp.c_str());
    std::string parse;
    std::getline(std::cin, parse);
    std::vector<double> variables;
    size_t n_elements = std::stoi(input);
    const char split_char = ',';
    std::istringstream split(parse);
    std::string each;
    while (std::getline(split, each, split_char)) {
        // for (std::string each; std::getline(split, each, split_char);
        variables.push_back(std::stod(each));
    }
    //;
    if (variables.size() != n_elements) {
        std::string msg("Expected to get: " + input + " elements, but got " +
                        std::to_string(variables.size()));
        throw std::runtime_error(msg);
    }
    return variables;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(3);
    spinner.start();
    Picking picker;
    picker.connectToRobot(nh);
    ros::Rate rate(1);
    ROS_INFO("Please specify the grapsing position========================");
    const std::vector<double> goal = parseInput("6");
    std::cout << "received value:\n";
    for (const auto x : goal) {
        std::cout << x << ", ";
    }
    std::cout << "\n";
    const auto pre_grasp = computePosition(goal, true);
    const auto grasp = computePosition(goal, false);
    std::vector<Picking::EEFPosition> movements = {pre_grasp, grasp};
    for (const auto &movement : movements) {
        picker.moveToPosition(movement);
    }
    return 0;
}

