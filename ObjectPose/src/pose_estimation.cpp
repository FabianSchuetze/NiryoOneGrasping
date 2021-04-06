#include "pose_estimation.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <sstream>

//typedef o3d::geometry::PointCloud Pointcloud;
////typedef std::shared_ptr<o3d::geometry::PointCloud> PointCloud::Ptr;
//using Ptr = std::shared_ptr<o3d::geometry::PointCloud>;

Eigen::Vector3d inline convert_color(const pcl::PointXYZRGB &point) {
    double r = static_cast<double>(point.r) / 255;
    double g = static_cast<double>(point.g) / 255;
    double b = static_cast<double>(point.b) / 255;
    return {r, g, b};
}

namespace ObjectPose {

void PoseEstimation::readMeshes(const std::filesystem::path& path) {
    std::string line;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::runtime_error("Files can't be opened");
    }
    while (std::getline(file, line)) {
        o3d::geometry::TriangleMesh mesh;
        bool success = o3d::io::ReadTriangleMesh(line, mesh);
        if (!success) {
            throw std::runtime_error("Could not read mesh file");
        }
        auto pcl = mesh.SamplePointsUniformly(1000);
        meshes.push_back(pcl);
    }
}

PoseEstimation::PoseEstimation(const std::filesystem::path& path) {
}

std::shared_ptr<o3d::geometry::PointCloud> PoseEstimation::toOpen3DPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_cloud) {
    std::shared_ptr<o3d::geometry::PointCloud> o3d_cloud =
        std::make_shared<o3d::geometry::PointCloud>();
    std::vector<Eigen::Vector3d> points, colors;
    points.reserve(pcl_cloud->size()), colors.reserve(pcl_cloud->size());
    for (const auto &point : *pcl_cloud) {
        points.emplace_back(point.x, point.x, point.x);
        colors.push_back(convert_color(point));
    }
    o3d_cloud->points_ = points;
    o3d_cloud->colors_ = colors;
    return o3d_cloud;
}

void PoseEstimation::findCluster(const Ptr& source) {
    std::vector<int> indices = source->ClusterDBSCAN(0.02, 100, false);
    auto it = std::max_element(indices.begin(), indices.end());
    if (*it == 0) {
        throw std::runtime_error("not possible");
    }
    clusters.resize(*it);
    for (std::size_t idx = 0; idx < clusters.size(); ++idx) {
        std::size_t cluster = indices[idx];
        Eigen::Vector3d point = source->points_[idx];
        Eigen::Vector3d color = source->points_[idx];
        clusters[cluster]->points_.push_back(point);
        clusters[cluster]->colors_.push_back(color);
    }
}

void PoseEstimation::callback(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input) {
    auto pcl = toOpen3DPointCloud(input);
    findCluster(pcl);
    // ROS_WARN_STREAM("The frame is " << frame);
}
} // namespace ObjectPose
