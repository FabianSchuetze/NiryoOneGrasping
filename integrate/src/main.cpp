#include "integrate.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <filesystem>
#include <pick_place/MoveJointsAction.h>
#include <ros/ros.h>

static constexpr std::size_t FREQUENCY(30);
static constexpr std::size_t QUEUE(20);

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
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    // std::pair<std::string, std::string> root("integrate/root", "");
    std::pair<std::string, std::string> camera("integrate/camera", "");
    std::pair<std::string, std::string> mover("integrate/move_joints_server",
                                              "");
    readParameters(nh, camera, mover);
    actionlib::SimpleActionClient<pick_place::MoveJointsAction> ac(mover.second,
                                                                   true);
    ROS_WARN_STREAM("Waiting for server " << mover.second << "to start");
    // wait for the action server to start
    ac.waitForServer(); // will wait for infinite time
    integration::Integration integrate(camera.second);
    ROS_INFO("Action server started, sending goal.");
    std::string STATE("SUCCEEDED");
    ros::Subscriber sub =
        nh.subscribe("/camera/depth_registered/points", QUEUE,
                     &integration::Integration::callback, &integrate);
    // send a goal to the action
    pick_place::MoveJointsGoal goal;
    ac.sendGoal(goal);
    ros::Rate rate(FREQUENCY);
    while (ros::ok() && STATE != ac.getState().toString()) {
        auto state = ac.getState();
        ROS_WARN_STREAM("The state is: " << state.toString());
        ros::spinOnce();
        rate.sleep();
    }
    sub.shutdown();
    auto scene = integrate.createScene();
    o3d::visualization::DrawGeometries({scene}, "Cluster");
}
