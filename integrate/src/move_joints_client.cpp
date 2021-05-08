#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
#include <new_pick_place/MoveJointsAction.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");
    std::string server_name("pick_place/move_joints");
    actionlib::SimpleActionClient<new_pick_place::MoveJointsAction> ac(server_name, true);
    ros::NodeHandle nh;
    ROS_WARN_STREAM("Waiting for action server " << server_name << "to start");
    ac.waitForServer();  // will wait for infinite time
    ROS_INFO("Action server started, sending goal.");
    new_pick_place::MoveJointsGoal goal;
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_WARN_STREAM("Action finished: " << state.toString());
    } else
        ROS_INFO("Action did not finish before the time out.");
    return 0;
}

