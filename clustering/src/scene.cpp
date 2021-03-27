#include "scene.hpp"
#include <pcl_ros/transforms.h>

static constexpr int TIME_THRESHOLD(10);

void Scene::callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input) {
    const std::string target("base_link");
    bool success = pcl_ros::transformPointCloud(target, *input, *cloud, listener);
    if (!success) {
        throw std::runtime_error("Cannot transform pointcloud");
    }
    last_callback = std::chrono::system_clock::now();
}

bool Scene::pointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out) const {
    const auto diff = std::chrono::system_clock::now() - last_callback;
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(diff);
    if (sec.count() > TIME_THRESHOLD) {
        ROS_WARN_STREAM("Last callback too old");
        return false;
    }
    out = cloud;
    return true;
}
