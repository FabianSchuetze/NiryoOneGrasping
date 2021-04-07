#include "pose_estimation.hpp"
#include <ros/ros.h>
#include <string>
#include <filesystem>
//using namespace ObjectPose;
static constexpr std::size_t QUEUE(10);

template <typename T>
void readParameters(const ros::NodeHandle& nh, T& t) 
{
    nh.getParam(t.first, t.second);
    if (t.second.empty()) {
        ROS_WARN_STREAM("Rosparam " << t.first << " not identified");
        throw std::runtime_error("Could not read all parameters");
    }
    ROS_WARN_STREAM("THe parameters for " << t.first << " is " << t.second);
}

template<typename T, typename... Args>
void readParameters(const ros::NodeHandle& nh, T& t, Args&... args) // recursive variadic function
{
    nh.getParam(t.first, t.second);
    if (t.second.empty()) {
        ROS_WARN_STREAM("Rosparam " << t.first << " not identified");
        throw std::runtime_error("Could not read all parameters");
    }
    ROS_WARN_STREAM("THe parameters for " << t.first << " is " << t.second);
    readParameters(nh, args...) ;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    std::pair<std::string, std::string> mesh("pose_estimation/location_meshes", "");
    std::pair<std::string, std::string> topic("pose_estimation/cluster_topic", "");
    readParameters(nh, mesh, topic);
    std::filesystem::path path("/home/fabian/.ros/segmented.pcd");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile (path, *cloud) == -1)
    {
      pcl::console::print_error (stdout, "[failed]\n");
      return 1;
    }
    //nh.getParam("/location_meshes", meshes);
    ObjectPose::PoseEstimation pose_estimation(mesh.second);
    pose_estimation.callback(cloud);
    //ros::Rate rate(1);
    ////nh.getParam("/cluster/topic", _topic);
    ////if (_topic.empty()) {
        ////ROS_WARN_STREAM("Rosparam /cluster/topic not defined");
        ////return 1;
    ////}
    ////ROS_WARN_STREAM("The name for the topic is " << _topic);
    //ros::Subscriber sub =
        //nh.subscribe(topic.first, QUEUE, &ObjectPose::PoseEstimation::callback, &pose_estimation);
    //ros::spin();
}
