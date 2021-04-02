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
using NiryoPose = std::pair<geometry_msgs::Point, niryo_one_msgs::RPY>;

struct EndEffectorPosition {
    NiryoPose pose;
    bool open;
};

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

std::vector<double> parseInput(const std::string& input) {
    const std::string tmp =
        "Enter" + input + " elements, separated by whitespace";
    ROS_INFO("%s", tmp.c_str());
    std::string parse;
    std::getline(std::cin, parse);
    std::vector<double> variables;
    size_t n_elements = std::stoi(input);
    const char split_char = ',';
    std::istringstream split(parse);
    for (std::string each; std::getline(split, each, split_char);
         variables.push_back(std::stod(each)))
        ;
    if (variables.size() != n_elements) {
        std::string msg("Expected to get: " + input + " elements, but got " +
                        std::to_string(variables.size()));
        throw std::runtime_error(msg);
    }
    return variables;
}

bool MoveJoints(NiryoClient& ac, const std::vector<double>& joints) {
    niryo_one_msgs::RobotMoveCommand cmd;
    cmd.cmd_type = 1;
    cmd.joints = joints;
    //cmd.position = pose.first;
    //cmd.rpy = pose.second;
    niryo_one_msgs::RobotMoveActionGoal action;
    action.goal.cmd = cmd;
    ROS_INFO("Sending command:");
    ROS_INFO("position: %.2f, %.2f, %.2f", cmd.joints[0], cmd.joints[1],
            cmd.joints[2]);
    ROS_INFO("rpy (r,p,y):  %.2f, %.2f, %2f", cmd.joints[3], cmd.joints[4],
            cmd.joints[5]);
    ac.sendGoal(action.goal);
    bool success = ac.waitForResult(ros::Duration(5.0));
    return success;
}

void positionGoal(NiryoClient& ac, const std::vector<double>& joints) {
    bool movement = MoveJoints(ac, joints);
    if (!movement) {
        ROS_WARN("Could not move the arm");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle n("~");
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Connecting to the robot ===========================================
    NiryoClient ac("/niryo_one/commander/robot_action/", true);
    establish_connection(ac);  // wait for the action server to start

    int toolID(13);
    setGripper(n, toolID);

    ros::Rate rate(1);
    ROS_INFO("Please specify the grapsing position========================");
    const std::vector<double> goal = parseInput("6");
    std::cout << "received value:\n";
    for (const auto x : goal) {
        std::cout << x << ", ";
    }
    std::cout << "\n";
    positionGoal(ac, goal);
    return 0;
}

