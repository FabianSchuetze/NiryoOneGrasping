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
    param saved_data("integrate/saved_data", "");
    utils::readParameters(nh, camera, debug, saved_data);
    integration::Integration integrate(camera.second, saved_data.second);
    integrate.readFiles();
    //integrate.convertPointCloudsToRGBD();
    auto scene = integrate.createScene();
    ROS_WARN_STREAM("finsished scene" << std::endl);
    std::filesystem::path save_location{integrate.paths.pointcloud / "final.pcd"};
    ROS_WARN_STREAM("The path is: " << save_location);
    bool success = integrate.save_pointcloud(*scene, save_location);
    ROS_WARN_STREAM("Success: " << success);
    o3d::visualization::DrawGeometries({scene}, "Cluster");
    //integrate.publishCloud(scene);
    //sub.shutdown();
    return 0;
}
