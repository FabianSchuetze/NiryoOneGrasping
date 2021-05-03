#include "integrate.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <filesystem>
#include <pick_place/MoveJointsAction.h>
#include <ros/ros.h>

static constexpr std::size_t FREQUENCY(40);
static constexpr std::size_t QUEUE(30);
static std::string SUCCEEDED("SUCCEEDED");

//TODO: Take from utils
template <typename... T>
void readParameters(const ros::NodeHandle &nh, T &... args) {
    auto read_parameters = [&](auto &t) {
        nh.getParam(t.first, t.second);
        if constexpr (std::is_same_v<decltype(t), std::string>) {
            if (t.second.empty()) {
                ROS_WARN_STREAM("Rosparam " << t.first << " not identified");
                throw std::runtime_error("Could not read all parameters");
            }
        }
        ROS_WARN_STREAM("The parameters for " << t.first << " is " << t.second);
    };
    (..., read_parameters(args));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    std::pair<std::string, bool> debug("integrate/debug", true);
    std::pair<std::string, std::string> camera("integrate/camera", "");
    std::pair<std::string, std::string> mover("integrate/move_joints_server",
                                              "");
    std::pair<std::string, std::string> cameraFrame("integrate/cameraFrame",
                                                    "");
    std::pair<std::string, std::string> publishTopic("integrate/publishTopic",
                                                     "");
    readParameters(nh, camera, mover, cameraFrame, publishTopic, debug);
    actionlib::SimpleActionClient<pick_place::MoveJointsAction> ac(mover.second,
                                                                   true);
    ROS_WARN_STREAM("Waiting for server " << mover.second << "to start");
    ac.waitForServer();
    ROS_WARN_STREAM("Connected to action server.");
    integration::Integration integrate(camera.second, cameraFrame.second,
                                       publishTopic.second, debug.second,
                                       nh);
    ros::Subscriber sub =
        nh.subscribe("/camera/depth_registered/points", QUEUE,
                     &integration::Integration::callback, &integrate);
    pick_place::MoveJointsGoal goal;
    ac.sendGoal(goal);
    ros::Rate rate(FREQUENCY);
    while (ros::ok() && SUCCEEDED != ac.getState().toString()) {
        auto state = ac.getState();
        ros::spinOnce();
        rate.sleep();
    }
    sub.shutdown();
    integrate.convertPointCloudsToRGBD();
    auto scene = integrate.createScene();
    o3d::visualization::DrawGeometries({scene}, "Cluster");
    integrate.publishCloud(scene);
    sub.shutdown();
    return 0;
}
