#include "pose_estimation.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <sstream>

// typedef o3d::geometry::PointCloud Pointcloud;
////typedef std::shared_ptr<o3d::geometry::PointCloud> PointCloud::Ptr;
// using Ptr = std::shared_ptr<o3d::geometry::PointCloud>;
//
static constexpr std::size_t N_POINTS(1000);
static constexpr std::size_t MAX_UINT(255);
using o3d::pipelines::registration::RegistrationResult;

// struct BestResult {
// int source_idx = -1;
// int target_idx = -1;
// RegistrationResult result;
// ObjectPose::Ptr source;
// ObjectPose::Ptr target;
//};

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

void PoseEstimation::findCluster(const Ptr &source) {
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
    std::for_each(clusters.begin(), clusters.end(),
                  [](auto &x) { x->EstimateNormals(); });
    // for (auto& cluster : clusters) {
    // cluster->EstimateNormals();
    //}
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
    // ROS_WARN_STREAM("The frame is " << frame);
}
} // namespace ObjectPose
