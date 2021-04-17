#include "scene.hpp"
#include <pcl_ros/transforms.h>


static constexpr int TIME_THRESHOLD(5);
static std::string BASE_LINK("base_link");
using namespace Clustering;

void Scene::callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input) {
    std::string frame = input->header.frame_id;
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!(frame == BASE_LINK)) {
        const std::string target("base_link");
        if (!pcl_ros::transformPointCloud(target, *input, *cloud, listener)) {
            ROS_WARN_STREAM("Cannot transform pointcloud");
            return;
        }
    } else {
        *cloud = *input;
    }
    last_callback = std::chrono::system_clock::now();
}

bool Scene::pointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out) {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto diff = std::chrono::system_clock::now() - last_callback;
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(diff);
    if (sec.count() > TIME_THRESHOLD) {
        ROS_DEBUG_STREAM("Last callback too old");
        return false;
    }
    out = cloud;
    return true;
}
