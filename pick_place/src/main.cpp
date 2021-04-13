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

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_place");
    ros::AsyncSpinner spinner(3);
    spinner.start();
    Picking picker;
    picker.connectToRobot();
    picker.connectToPositionServer();
    geometry_msgs::Point goal;
    std::chrono::milliseconds sec(500);
    while (picker.obtainPose(goal)) {
        ROS_INFO_STREAM("received value:\n"
                        << goal.x << ", " << goal.y << ", " << goal.z);
        const Picking::EndEffectorPosition pre_grasp =
            Picking::computePreGrasp(goal);
        const Picking::EndEffectorPosition grasp = Picking::computeGrasp(goal);
        const Picking::EndEffectorPosition close = Picking::Close(goal);
        const Picking::EndEffectorPosition post_grasp = Picking::PostGrasp(goal);
        const Picking::EndEffectorPosition pre_final = Picking::PreFinal();
        const Picking::EndEffectorPosition open = Picking::Final();
        const Picking::EndEffectorPosition rest_position = Picking::Rest();
        std::vector<Picking::EndEffectorPosition> movements = {
            pre_grasp, grasp, close, post_grasp, pre_final, open, rest_position};
        for (const auto &movement : movements) {
            picker.moveToPosition(movement);
            std::this_thread::sleep_for(sec);
        }
        std::this_thread::sleep_for(sec);
    }
    return 0;
}
