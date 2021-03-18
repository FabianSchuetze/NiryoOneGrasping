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
#include <pose_detection/BroadcastPoseAction.h>

#include <sstream>

#include "picking.hpp"
#include "ros/ros.h"
constexpr static int TOOL_ID(13);

// EndEffectorPosition computePreGrasp(const std::vector<double>& goal) {
// geometry_msgs::Point p;
// p.x = goal[0];
// p.y = goal[1];
// p.z = goal[2] + 0.15;
// if (p.z < 0.135) {
// throw std::runtime_error("Z values cannot be lower than 0.135");
//}
// niryo_one_msgs::RPY rot;
// rot.roll = 0;
// rot.pitch = 1.5;
// rot.yaw = 0;
// NiryoPose pose1(p, rot);
// EndEffectorPosition eef;
// eef.pose = pose1;
// eef.open = false;
// return eef;
//}

// EndEffectorPosition computeGrasp(const std::vector<double>& goal) {
// geometry_msgs::Point p;
// p.x = goal[0];
// p.y = goal[1];
// p.z = goal[2] + 0.135;
// if (p.z < 0.135) {
// throw std::runtime_error("Z values cannot be lower than 0.135");
//}
// niryo_one_msgs::RPY rot;
// rot.roll = 0;
// rot.pitch = 1.5;
// rot.yaw = 0;
// NiryoPose pose1(p, rot);
// EndEffectorPosition eef;
// eef.pose = pose1;
// eef.open = true;
// return eef;
//}

// std::vector<double> parseInput(const std::string& input) {
// const std::string tmp =
//"Enter" + input + " elements, separated by whitespace";
// ROS_INFO("%s", tmp.c_str());
// std::string parse;
// std::getline(std::cin, parse);
// std::vector<double> variables;
// size_t n_elements = std::stoi(input);
// const char split_char = ',';
// std::istringstream split(parse);
// for (std::string each; std::getline(split, each, split_char);
// variables.push_back(std::stod(each)))
//;
// if (variables.size() != n_elements) {
// std::string msg("Expected to get: " + input + " elements, but got " +
// std::to_string(variables.size()));
// throw std::runtime_error(msg);
//}
// return variables;
//}

// bool GripperAperture(NiryoClient& ac, bool open) {
// niryo_one_msgs::ToolCommand tcmd;
// if (open) {
// tcmd.cmd_type = 1;
//} else {
// tcmd.cmd_type = 2;
//}
// tcmd.gripper_open_speed = 200;
// tcmd.tool_id = 13;
// niryo_one_msgs::RobotMoveActionGoal action;
// action.goal.cmd.cmd_type = 6;
// action.goal.cmd.tool_cmd = tcmd;
// ac.sendGoal(action.goal);
// bool success = ac.waitForResult(ros::Duration(5.0));
// return success;
//}:

// niryo_one_msgs::RobotMoveCommand cmd;
// cmd.cmd_type = 2;
// cmd.position = pose.first;
// cmd.rpy = pose.second;
// niryo_one_msgs::RobotMoveActionGoal action;
// action.goal.cmd = cmd;
// ROS_INFO("Sending command:");
// ROS_INFO("position: %.2f, %.2f, %.2f", cmd.position.x, cmd.position.y,
// cmd.position.z);
// ROS_INFO("rpy (r,p,y):  %.2f, %.2f, %2f", cmd.rpy.roll, cmd.rpy.pitch,
// cmd.rpy.yaw);
// ac.sendGoal(action.goal);
// bool success = ac.waitForResult(ros::Duration(5.0));
// return success;
//}

// void positionGoal(NiryoClient& ac, const EndEffectorPosition& eef) {
// bool movement = MoveEEF(ac, eef.pose);
// if (!movement) {
// ROS_WARN("Could not move the arm");
//}
// bool aperture = GripperAperture(ac, eef.open);
// if (!aperture) {
// ROS_WARN("Could not open the gripper");
//}
//}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle n("~");
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Connecting to the robot ===========================================
    //Picking::NiryoClient ac("/niryo_one/commander/robot_action/", true);
    //Picking::establish_connection(ac); // wait for the action server to start

    //Picking::setGripper(n, TOOL_ID);
    actionlib::SimpleActionClient<pose_detection::BroadcastPoseAction> ac2(
        "poseaction", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac2.waitForServer(); // will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    pose_detection::BroadcastPoseGoal goal;
    // goal.order = 20;
    ac2.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac2.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        const auto result = ac2.getResult();
        geometry_msgs::TransformStamped out = result->pose;
        actionlib::SimpleClientGoalState state = ac2.getState();
        std::cout << "The translation is: " << out.transform.translation.x;
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else
        ROS_INFO("Action did not finish before the time out.");

    // exit
    return 0;

    // ros::Rate rate(1);
    ////ROS_INFO_STREAM("Please specify the grapsing
    ///position===================");
    // const std::vector<double> goal = parseInput("6");
    // std::cout << "received value:\n";
    // for (const auto x : goal) {
    // std::cout << x << ", ";
    //}
    // std::cout << "\n";
    //// std::vector<EndEffectorPosition> movements;
    //// movements.reserve(2);
    // const EndEffectorPosition pre_grasp = computePreGrasp(goal);
    // const EndEffectorPosition grasp = computeGrasp(goal);
    // std::vector<EndEffectorPosition> movements = {pre_grasp, grasp};
    //// movements.push_back(pre_grasp);
    //// movements.push_back(grasp);
    // for (const auto &movement : movements) {
    // positionGoal(ac, movement);
    //}
    return 0;
}
