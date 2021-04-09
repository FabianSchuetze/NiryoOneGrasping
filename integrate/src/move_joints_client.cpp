#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
#include <pick_place/MoveJointsAction.h>
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");

    // create the action client
    // true causes the client to spin its own thread
    std::string server_name("pick_place/move_joints");
    actionlib::SimpleActionClient<pick_place::MoveJointsAction> ac(server_name, true);
    ros::NodeHandle nh;

    ROS_WARN_STREAM("Waiting for action server " << server_name << "to start");
    // wait for the action server to start
    ac.waitForServer();  // will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    pick_place::MoveJointsGoal goal;
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_WARN_STREAM("Action finished: " << state.toString());
    } else
        ROS_INFO("Action did not finish before the time out.");

    // exit
    return 0;
}

