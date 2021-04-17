#include "pose_estimation.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <sstream>
#include <std_msgs/Header.h>

static constexpr std::size_t N_POINTS(1000);
static constexpr std::size_t MAX_UINT(255);
using o3d::pipelines::registration::RegistrationResult;
namespace registration = o3d::pipelines::registration;

void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const registration::RegistrationResult &result) {
    std::shared_ptr<o3d::geometry::PointCloud> source_transformed_ptr(
        new o3d::geometry::PointCloud);
    std::shared_ptr<o3d::geometry::PointCloud> target_ptr(
        new o3d::geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(result.transformation_);
    std::stringstream ss;
    ss << "Registration Result, fitness: " << result.fitness_;
    o3d::visualization::DrawGeometries({source_transformed_ptr, target_ptr},
                                       ss.str());
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
        ROS_WARN_STREAM("The file is: " << line);
        o3d::geometry::TriangleMesh mesh;
        bool success = o3d::io::ReadTriangleMesh(line, mesh);
        if (!success) {
            throw std::runtime_error("Could not read mesh file");
        }
        meshes.emplace_back(std::move(line), std::move(mesh));
        // auto pcl = mesh.SamplePointsUniformly(N_POINTS);
        // pcl->EstimateNormals();
        // meshes.push_back(pcl);
    }
}

PoseEstimation::PoseEstimation(const std::filesystem::path &path,
                               const std::string &topic, ros::NodeHandle &node)
    : callback_received(0) {
    readMeshes(path);
    if (topic.empty()) {
        ROS_ERROR_STREAM("The topic for publishing poses is empty");
        throw std::runtime_error("Empty topic for publishing poses");
    }
    publisher = node.advertise<geometry_msgs::PoseArray>(topic, 1, true);
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

std::vector<Ptr> PoseEstimation::findCluster(const Ptr &source) {
    std::vector<Ptr> available_clusters;
    std::vector<int> indices = source->ClusterDBSCAN(0.02, 100, false);
    std::vector<std::size_t> points(indices.size());
    std::iota(points.begin(), points.end(), 0);
    ROS_WARN_STREAM("Found number of indices : " << indices.size());
    auto it = std::max_element(indices.begin(), indices.end());
    ROS_WARN_STREAM("The number of clusters is : " << *it);
    if (*it == -1) {
        ROS_ERROR_STREAM("Did not find any clusters");
        throw std::runtime_error("Did not find any clusters");
    }
    for (int cluster_idx = 0; cluster_idx <= *it; ++cluster_idx) {
        std::vector<std::size_t> cluster;
        std::copy_if(points.begin(), points.end(), std::back_inserter(cluster),
                     [&](std::size_t i) { return indices[i] == cluster_idx; });
        auto cloud = source->SelectByIndex(cluster);
        cloud->EstimateNormals();
        available_clusters.push_back(cloud);
    }
    return available_clusters;
    // for (auto &cluster : clusters) {
    // o3d::visualization::DrawGeometries({cluster}, "Cluster");
    //}
}

RegistrationResult PoseEstimation::globalRegistration(const Ptr &source,
                                                      const Ptr &target) {
    registration::RegistrationResult best_result;
    double max_fitness(0.0);
    const auto source_fpfh = registration::ComputeFPFHFeature(
        *source, o3d::geometry::KDTreeSearchParamHybrid(0.05, 100));
    const auto target_fpfh = registration::ComputeFPFHFeature(
        *target, o3d::geometry::KDTreeSearchParamHybrid(0.05, 100));
    std::vector<
        std::reference_wrapper<const registration::CorrespondenceChecker>>
        correspondence_checker;
    auto correspondence_checker_edge_length =
        registration::CorrespondenceCheckerBasedOnEdgeLength(0.92);
    auto correspondence_checker_distance =
        registration::CorrespondenceCheckerBasedOnDistance(1.5 * 0.01);
    auto correspondence_checker_normal =
        registration::CorrespondenceCheckerBasedOnNormal(0.52359878);
    correspondence_checker.emplace_back(correspondence_checker_edge_length);
    correspondence_checker.emplace_back(correspondence_checker_distance);
    correspondence_checker.emplace_back(correspondence_checker_normal);
    bool mutual_filter(true);
    for (int i = 0; i < 3; ++i) {
        auto preliminary = registration::RegistrationRANSACBasedOnFeatureMatching(
            *source, *target, *source_fpfh, *target_fpfh, mutual_filter, 1.5 * 0.05,
            registration::TransformationEstimationPointToPoint(false), 4,
            correspondence_checker,
            registration::RANSACConvergenceCriteria(5000000, 0.999999));
        auto registration_result = registration::RegistrationICP(
            *source, *target, 0.02, preliminary.transformation_);
        if (registration_result.fitness_ > max_fitness) {
            max_fitness = registration_result.fitness_;
            best_result = registration_result;
        }
    }
    // VisualizeRegistration(*source, *target, registration_result);
    return best_result;
}

PoseEstimation::BestResult PoseEstimation::estimateTransformations(
    std::vector<std::pair<std::string, o3d::geometry::TriangleMesh>> &sources,
    std::vector<Ptr> &targets) {
    double best(0.3);
    BestResult best_result;
    ROS_WARN_STREAM("Target and Source size: " << targets.size() << ", "
                                               << sources.size());
    for (size_t t_idx = 0; t_idx < targets.size(); ++t_idx) {
        ROS_WARN_STREAM("Doing Target " << t_idx);
        const Ptr &potential_target = targets[t_idx];
        std::size_t n_points = potential_target->points_.size();
        for (size_t s_idx = 0; s_idx < sources.size(); ++s_idx) {
            ROS_WARN_STREAM("Doing Source " << t_idx);
            auto [name, mesh] = sources[s_idx];
            auto pcd = mesh.SamplePointsUniformly(n_points);
            pcd->EstimateNormals();
            auto result = globalRegistration(pcd, potential_target);
            ROS_WARN_STREAM("Fittness of the result is:" << result.fitness_);
            if (result.fitness_ > best) {
                best_result.source_idx = s_idx;
                best_result.source_name = name;
                best_result.target_idx = t_idx;
                best_result.result = result;
                best_result.source = pcd;
                best_result.target = potential_target;
                best = result.fitness_;
            }
        }
    }
    return best_result;
}

std::vector<PoseEstimation::BestResult>
PoseEstimation::estimateTransformations() {
    std::vector<std::pair<std::string, o3d::geometry::TriangleMesh>> sources(
        meshes.begin(), meshes.end());
    std::vector<Ptr> targets(clusters.begin(), clusters.end());
    std::vector<BestResult> results;
    bool found(false);
    do {
        BestResult result = estimateTransformations(sources, targets);
        found = (result.source_idx != -1) and (result.target_idx != -1);
        if (found) {
            VisualizeRegistration(*result.source, *result.target,
                                  result.result);
            results.push_back(result);
            sources.erase(sources.begin() + result.source_idx);
            targets.erase(targets.begin() + result.target_idx);
        }
    } while (found);
    return results;
}

void PoseEstimation::publishTransforms(const std::vector<BestResult> &results) {
    geometry_msgs::PoseArray poses;
    std_msgs::Header header;
    header.frame_id = "base_link";
    header.seq = callback_received;
    poses.header = header;
    for (auto const &result : results) {
        ROS_WARN_STREAM("The transform of: " << result.source_name << " is\n"
                                             << result.result.transformation_);
        geometry_msgs::Pose pose;
        Eigen::Affine3d transform;
        transform = result.result.transformation_;
        tf::poseEigenToMsg(transform, pose);
        poses.poses.push_back(pose);
    }
    publisher.publish(poses);
    //ros::spinOnce();
}


void PoseEstimation::callback(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input) {
    if (callback_received > input->header.seq) {
        return;
    }
    ++callback_received;
    auto pcl = toOpen3DPointCloud(input);
    clusters = findCluster(pcl);
    std::vector<BestResult> results = estimateTransformations();
    publishTransforms(results);
}
} // namespace ObjectPose
