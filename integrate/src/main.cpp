#include "integrate.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <filesystem>
#include <new_pick_place/MoveJointsAction.h>
#include <ros/ros.h>
#include <utils/utils.hpp>

using param = std::pair<std::string, std::string>;
static constexpr std::size_t FREQUENCY(40);
static constexpr std::size_t QUEUE(30);
static std::string SUCCEEDED("SUCCEEDED");

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    std::pair<std::string, bool> debug("integrate/debug", true);
    param camera("integrate/camera", "");
    param mover("integrate/move_joints_server", "");
    param cameraFrame("integrate/cameraFrame", "");
    param publishTopic("integrate/publishTopic", "");
    utils::readParameters(nh, camera, mover, cameraFrame, publishTopic, debug);
    actionlib::SimpleActionClient<new_pick_place::MoveJointsAction> ac(mover.second,
                                                                   true);
    ROS_WARN_STREAM("Waiting for server " << mover.second << "to start");
    ac.waitForServer();
    ROS_WARN_STREAM("Connected to action server.");
    integration::Integration integrate(camera.second, cameraFrame.second,
                                       publishTopic.second, debug.second, nh);
    ros::Subscriber sub =
        nh.subscribe("/camera/depth_registered/points", QUEUE,
                     &integration::Integration::callback, &integrate);
    new_pick_place::MoveJointsGoal goal;
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
