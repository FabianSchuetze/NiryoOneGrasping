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

typedef actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>
    NiryoClient;

void establish_connection(NiryoClient& ac) {
    ROS_INFO("Connecting to robot  ========================");
    size_t attempts(0);
    while (!ac.waitForServer(ros::Duration(3.0))) {
        ROS_WARN("  Error connecting to Robot. Trying again");
        ++attempts;
        if (attempts > 10)
            throw std::runtime_error("Could not achieve connection");
    }
    ROS_INFO("  Robot Connection established");
}

void setGripper(ros::NodeHandle& node, int toolID) {
    ROS_INFO("Setting gripper");
    ros::ServiceClient changeToolClient_;
    changeToolClient_ =
        node.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/change_tool/");
    niryo_one_msgs::SetInt srv;
    srv.request.value = toolID;
    while (!changeToolClient_.call(srv)) {
        ROS_WARN("Could not set the tool type. Trying again in one second");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Success");
}

std::vector<float> parseInput(const std::string& input) {
    const std::string tmp =
        "Enter" + input + " elements, separated by whitespace";
    ROS_INFO("%s", tmp.c_str());
    std::string parse;
    std::getline(std::cin, parse);
    std::vector<float> variables;
    const char split_char = ':';
    std::istringstream split(parse);
    for (std::string each; std::getline(split, each, split_char);
         variables.push_back(std::stof(each)))
        ;
    return variables;
}

bool positionGoal(NiryoClient& ac) {
    std::vector<float> positions = parseInput("7");
    geometry_msgs::Pose pose_quat;
    pose_quat.position.x = positions[0];
    pose_quat.position.y = positions[1];  // for better demonstration purposes
    pose_quat.position.z = positions[2];
    pose_quat.orientation.x = positions[3];
    pose_quat.orientation.y = positions[4];
    pose_quat.orientation.z = positions[5];
    pose_quat.orientation.w = positions[6];
    niryo_one_msgs::RobotMoveCommand cmd;
    cmd.cmd_type = 8;
    cmd.pose_quat = pose_quat;
    ROS_INFO("  Sending command :");
    ROS_INFO("    position: %f, %f, %f", cmd.pose_quat.position.x,
             cmd.pose_quat.position.y, cmd.pose_quat.position.z);
    ROS_INFO("    orientation (x,y,z,w):  %f, %f, %f, %f",
             cmd.pose_quat.orientation.x, cmd.pose_quat.orientation.y,
             cmd.pose_quat.orientation.z, cmd.pose_quat.orientation.w);
    niryo_one_msgs::RobotMoveActionGoal action;
    action.goal.cmd = cmd;
    ac.sendGoal(action.goal);
    bool success = ac.waitForResult(ros::Duration(5.0));
    return success;
}

bool moveGripper(NiryoClient& ac, bool open) {
    niryo_one_msgs::ToolCommand tcmd;
    if (open) {
        tcmd.cmd_type = 1;
    } else {
        tcmd.cmd_type = 2;
    }
    tcmd.gripper_open_speed = 100;
    tcmd.tool_id = 13;
    niryo_one_msgs::RobotMoveActionGoal action;
    action.goal.cmd.cmd_type = 6;
    action.goal.cmd.tool_cmd = tcmd;
    ac.sendGoal(action.goal);
    bool success = ac.waitForResult(ros::Duration(10.0));
    return success;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Connecting to the robot ===========================================
    NiryoClient ac("/niryo_one/commander/robot_action/", true);
    establish_connection(ac);
    // wait for the action server to start

    int toolID(13);
    setGripper(n, toolID);

    ros::Rate rate(1);
    while (n.ok()) {
        bool success = positionGoal(ac);
        if (!success) {
            ROS_WARN("Could not satisfy the pose");
        }
        ROS_INFO("Send gripper command (open) ========================");
        success = moveGripper(ac, true);
        if (!success) {
            ROS_WARN("Could not open the gripper");
        }
        ROS_INFO("Send gripper command (close) ========================");
        success = moveGripper(ac, false);
        if (!success) {
            ROS_WARN("Could not close the gripper");
        }
        ROS_INFO("  Press enter to send ...");
        success = positionGoal(ac);
        if (!success) {
            ROS_WARN("Could not satisfy the pose");
        }
        rate.sleep();
    }
    return 0;
}

