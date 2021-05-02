#include "pose_estimation.hpp"
#include <filesystem>
#include <ros/ros.h>
#include <string>
static constexpr std::size_t QUEUE(10);
using param = std::pair<std::string, std::string>;

//TODO: Replace with utils
template <typename... T>
void readParameters(const ros::NodeHandle &nh, T &... args) {
    auto read_parameters = [&](auto &t) {
        nh.getParam(t.first, t.second);
        if (t.second.empty()) {
            ROS_ERROR_STREAM("Rosparam " << t.first << " not identified");
            throw std::runtime_error("Could not read all parameters");
        }
        ROS_WARN_STREAM("The parameters for " << t.first << " is " << t.second);
    };
    (..., read_parameters(args));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    param mesh("pose_estimation/location_meshes", "");
    param incoming_clusters("pose_estimation/segmented", "");
    param estimated_poses("pose_estimation/estimated_poses", "");
    readParameters(nh, mesh, incoming_clusters, estimated_poses);
    //std::filesystem::path path("/home/fabian/.ros/segmented_old.pcd");
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        //new pcl::PointCloud<pcl::PointXYZRGB>);
    //if (pcl::io::loadPCDFile(path, *cloud) == -1) {
        //pcl::console::print_error(stdout, "[failed]\n");
        //return 1;
    //}
    ObjectPose::PoseEstimation pose_estimation(mesh.second,
                                               estimated_poses.second, nh);
     //pose_estimation.callback(cloud);
    ros::Subscriber sub =
        nh.subscribe(incoming_clusters.second, QUEUE,
                     &ObjectPose::PoseEstimation::callback, &pose_estimation);
    ros::spin();
}
