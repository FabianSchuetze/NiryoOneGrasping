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

#include <sstream>

#include "picking.hpp"
#include "ros/ros.h"
constexpr static int TOOL_ID(13);

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle n("~");
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Connecting to the robot ===========================================
    Picking::NiryoClient ac("/niryo_one/commander/robot_action/", true);
    Picking::establish_connection(
        ac, "robot"); // wait for the action server to start
    Picking::setGripper(n, TOOL_ID);
    Picking::PoseClient ac2("poseaction", true);
    Picking::establish_connection(ac2, "pose_estimate");
    std::vector<double> goal = Picking::obtainPose(ac2);

    // ROS_INFO("Action server started, sending goal.");
    ros::Rate rate(1);
    std::cout << "received value:\n";
    for (const auto x : goal) {
        std::cout << x << ", ";
    }
    std::cout << "\n";
    // std::vector<EndEffectorPosition> movements;
    // movements.reserve(2);
    const Picking::EndEffectorPosition pre_grasp =
        Picking::computePreGrasp(goal);
    const Picking::EndEffectorPosition grasp = Picking::computeGrasp(goal);
    std::vector<Picking::EndEffectorPosition> movements = {pre_grasp, grasp};
    // movements.push_back(pre_grasp);
    // movements.push_back(grasp);
    for (const auto &movement : movements) {
        Picking::positionGoal(ac, movement);
    }
    return 0;
}
