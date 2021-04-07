#include "pose_estimation.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <sstream>

static constexpr std::size_t N_POINTS(1000);
static constexpr std::size_t MAX_UINT(255);
using o3d::pipelines::registration::RegistrationResult;

void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    std::shared_ptr<o3d::geometry::PointCloud> source_transformed_ptr(
        new o3d::geometry::PointCloud);
    std::shared_ptr<o3d::geometry::PointCloud> target_ptr(
        new o3d::geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(Transformation);
    o3d::visualization::DrawGeometries({source_transformed_ptr, target_ptr},
                                       "Registration result");
}

Eigen::Vector3d inline convert_color(const pcl::PointXYZRGB &point) {
    double r = static_cast<double>(point.r) / MAX_UINT;
    double g = static_cast<double>(point.g) / MAX_UINT;
    double b = static_cast<double>(point.b) / MAX_UINT;
    return {r, g, b};
}

namespace ObjectPose {

void PoseEstimation::readMeshes(const std::filesystem::path &path) {
    std::string line;
    ROS_WARN_STREAM("Trying to open file " << path);
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Files can't be opened");
    }
    while (std::getline(file, line)) {
        o3d::geometry::TriangleMesh mesh;
        bool success = o3d::io::ReadTriangleMesh(line, mesh);
        if (!success) {
            throw std::runtime_error("Could not read mesh file");
        }
        auto pcl = mesh.SamplePointsUniformly(N_POINTS);
        pcl->EstimateNormals();
        meshes.push_back(pcl);
    }
}

PoseEstimation::PoseEstimation(const std::filesystem::path &path) {
    readMeshes(path);
}

std::shared_ptr<o3d::geometry::PointCloud> PoseEstimation::toOpen3DPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_cloud) {
    ROS_WARN_STREAM("The input point cloud has size " << pcl_cloud->size());
    std::shared_ptr<o3d::geometry::PointCloud> o3d_cloud =
        std::make_shared<o3d::geometry::PointCloud>();
    std::vector<Eigen::Vector3d> points, colors;
    points.reserve(pcl_cloud->size()), colors.reserve(pcl_cloud->size());
    for (const auto &point : *pcl_cloud) {
        points.emplace_back(point.x, point.y, point.z);
        colors.push_back(convert_color(point));
    }
    ROS_WARN_STREAM("The number of copied points " << points.size());
    o3d_cloud->points_ = points;
    o3d_cloud->colors_ = colors;
    return o3d_cloud;
}

//void PoseEstimation::converToOpen3d(
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> tmp_clusters) {
    //for (const auto &cluster : tmp_clusters) {
        //auto o3d_cluster = toOpen3DPointCloud(cluster);
        //clusters.push_back(o3d_cluster);
    //}
//}

void PoseEstimation::findCluster(const Ptr &source) {
    o3d::visualization::DrawGeometries({source}, "Begin Cluster");
    std::vector<int> indices = source->ClusterDBSCAN(0.02, 100, false);
    std::vector<std::size_t> points(indices.size());
    std::iota(points.begin(), points.end(), 0);
    ROS_WARN_STREAM("Found number of indices : " << indices.size());
    auto it = std::max_element(indices.begin(), indices.end());
    ROS_WARN_STREAM("The number of clusters is : " << *it);
    if (*it == -1) {
        throw std::runtime_error("not possible");
    }
    ROS_WARN_STREAM("The size of the cluster vector is : " << clusters.size());
    for (int cluster_idx = 0; cluster_idx <= *it; ++cluster_idx) {
        std::vector<std::size_t> cluster;
        std::copy_if(points.begin(), points.end(), std::back_inserter(cluster),
                     [&](std::size_t i) { return indices[i] == cluster_idx; });
        auto cloud = source->SelectByIndex(cluster);
        cloud->EstimateNormals();
        clusters.push_back(cloud);
    }
    for (auto &cluster : clusters) {
        o3d::visualization::DrawGeometries({cluster}, "Cluster");
    }
}

RegistrationResult PoseEstimation::globalRegistration(
    const Ptr &source, const Ptr &target,
    const std::shared_ptr<o3d::pipelines::registration::Feature> &source_fpfh,
    const std::shared_ptr<o3d::pipelines::registration::Feature> &target_fpfh) {
    std::vector<std::reference_wrapper<
        const o3d::pipelines::registration::CorrespondenceChecker>>
        correspondence_checker;
    auto correspondence_checker_edge_length =
        o3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
            0.9);
    auto correspondence_checker_distance =
        o3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(
            1.5 * 0.75);
    // auto correspondence_checker_normal =
    // pipelines::registration::CorrespondenceCheckerBasedOnNormal(
    // 0.52359878);
    correspondence_checker.emplace_back(correspondence_checker_edge_length);
    correspondence_checker.emplace_back(correspondence_checker_distance);
    bool mutual_filter(true);
    auto result =
        o3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
            *source, *target, *source_fpfh, *target_fpfh, mutual_filter,
            1.5 * 0.75,
            o3d::pipelines::registration::TransformationEstimationPointToPoint(
                false),
            4, correspondence_checker,
            o3d::pipelines::registration::RANSACConvergenceCriteria(5000000,
                                                                    0.9999));
    return result;
}

RegistrationResult PoseEstimation::estimateTransformation(const Ptr &source,
                                                          const Ptr &target) {
    const auto source_features =
        o3d::pipelines::registration::ComputeFPFHFeature(
            *source, o3d::geometry::KDTreeSearchParamHybrid(0.05, 100));
    const auto target_features =
        o3d::pipelines::registration::ComputeFPFHFeature(
            *target, o3d::geometry::KDTreeSearchParamHybrid(0.05, 100));
    auto result =
        globalRegistration(source, target, source_features, target_features);
    return result;
}

PoseEstimation::BestResult
PoseEstimation::estimateTransformations(std::vector<Ptr> &sources,
                                        std::vector<Ptr> &targets) {
    double best(0.3);
    BestResult best_result;
    for (std::size_t target_idx = 0; target_idx < meshes.size(); ++target_idx) {
        const auto potential_target = targets[target_idx];
        for (std::size_t source_idx = 0; source_idx < clusters.size();
             ++source_idx) {
            const auto source = sources[source_idx];
            auto result = estimateTransformation(source, potential_target);
            if (result.fitness_ > best) {
                best_result.source_idx = source_idx;
                best_result.target_idx = target_idx;
                best_result.result = result;
                best_result.source = source;
                best_result.target = potential_target;
                sources.erase(sources.begin() + source_idx);
                targets.erase(targets.begin() + target_idx);
                best = result.fitness_;
            }
        }
    }
    return best_result;
}

void PoseEstimation::estimateTransformations() {
    std::vector<Ptr> sources(meshes.begin(), meshes.end());
    std::vector<Ptr> targets(clusters.begin(), clusters.end());
    bool found(false);
    do {
        BestResult result = estimateTransformations(sources, targets);
        found = (result.source_idx != -1) & (result.target_idx != -1);
        if (found) {
            VisualizeRegistration(*result.source, *result.target,
                                  result.result.transformation_);
        }
        // visualize the result
    } while (found);
}

void PoseEstimation::callback(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input) {
    auto pcl = toOpen3DPointCloud(input);
    findCluster(pcl);
    estimateTransformations();
}
} // namespace ObjectPose
