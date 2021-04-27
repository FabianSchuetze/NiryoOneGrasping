#include "pose_estimation.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <object_pose/positions.h>
#include <sstream>
#include <std_msgs/Header.h>
// test

static constexpr std::size_t N_POINTS(1000);
static constexpr std::size_t MAX_UINT(255);
static constexpr float SCALE(0.1);
static constexpr float MIN_DENSITY(0.02);
static constexpr std::size_t MIN_POINTS(100);
static constexpr std::size_t MAX_REPEATS(5000000);
static constexpr float CERTAINTY(0.99999);
static constexpr float GRAPH(0.92);

using o3d::pipelines::registration::RegistrationResult;
namespace registration = o3d::pipelines::registration;

Eigen::Vector3d inline convert_color(const pcl::PointXYZRGB &point) {
    double r = static_cast<double>(point.r) / MAX_UINT;
    double g = static_cast<double>(point.g) / MAX_UINT;
    double b = static_cast<double>(point.b) / MAX_UINT;
    return {r, g, b};
}

namespace ObjectPose {

void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const registration::RegistrationResult &result) {
    auto sourcep = std::make_shared<o3d::geometry::PointCloud>(source);
    auto targetp = std::make_shared<o3d::geometry::PointCloud>(target);
    auto origp = o3d::geometry::TriangleMesh::CreateCoordinateFrame(SCALE);
    auto framep = o3d::geometry::TriangleMesh::CreateCoordinateFrame(SCALE);
    framep->Transform(result.transformation_);
    sourcep->Transform(result.transformation_);
    std::stringstream ss;
    double roll = ObjectPose::calculateRoll(result.transformation_);
    double yaw = ObjectPose::calculateYaw(result.transformation_);
    double pitch = ObjectPose::calculatePitch(result.transformation_);
    ss << "Registration Result, fitness, roll, yaw, pitch " << result.fitness_
       << ", " << roll << ", " << yaw << ", " << pitch;
    o3d::visualization::DrawGeometries({sourcep, targetp, origp, framep},
                                       ss.str());
}

double calculateRoll(const Eigen::Matrix4d &transformation) {

    Eigen::Matrix3d tmp = transformation.block(0, 0, 3, 3);
    Eigen::Quaternion<double> quat(tmp);
    double first = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
    double second = 1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z());
    double roll = std::atan2(first, second);
    return roll;
}
double calculateYaw(const Eigen::Matrix4d &transformation) {

    Eigen::Matrix3d tmp = transformation.block(0, 0, 3, 3);
    Eigen::Quaternion<double> quat(tmp);
    double first = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
    double second = 1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z());
    double yaw = std::atan2(first, second);
    return yaw;
}

double calculatePitch(const Eigen::Matrix4d &transformation) {

    Eigen::Matrix3d tmp = transformation.block(0, 0, 3, 3);
    Eigen::Quaternion<double> quat(tmp);
    double first = 2 * (quat.w() * quat.y() - quat.y() * quat.x());
    double pitch = std::asin(first);
    return pitch;
}
void PoseEstimation::readMeshes(const std::filesystem::path &path) {
    std::string line;
    ROS_INFO_STREAM("Trying to open file " << path);
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
    publisher = node.advertise<object_pose::positions>(topic, 1, true);
}

std::shared_ptr<o3d::geometry::PointCloud> PoseEstimation::toOpen3DPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_cloud) {
    ROS_INFO_STREAM("The input point cloud has size " << pcl_cloud->size());
    std::shared_ptr<o3d::geometry::PointCloud> o3d_cloud =
        std::make_shared<o3d::geometry::PointCloud>();
    std::vector<Eigen::Vector3d> points, colors;
    points.reserve(pcl_cloud->size()), colors.reserve(pcl_cloud->size());
    for (const auto &point : *pcl_cloud) {
        points.emplace_back(point.x, point.y, point.z);
        colors.push_back(convert_color(point));
    }
    ROS_INFO_STREAM("The number of copied points " << points.size());
    o3d_cloud->points_ = points;
    o3d_cloud->colors_ = colors;
    return o3d_cloud;
}

std::vector<Ptr> PoseEstimation::findCluster(const Ptr &source) {
    std::vector<Ptr> available_clusters;
    std::vector<int> indices =
        source->ClusterDBSCAN(MIN_DENSITY, MIN_POINTS, false);
    std::vector<std::size_t> points(indices.size());
    std::iota(points.begin(), points.end(), 0);
    ROS_INFO_STREAM("Found number of indices : " << indices.size());
    auto it = std::max_element(indices.begin(), indices.end());
    ROS_INFO_STREAM("The number of clusters is : " << *it);
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

std::pair<double, RegistrationResult>
PoseEstimation::globalRegistration(const Ptr &source, const Ptr &target) {
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
        registration::CorrespondenceCheckerBasedOnEdgeLength(GRAPH);
    correspondence_checker.emplace_back(correspondence_checker_edge_length);
    bool mutual_filter(true);
    o3d::pipelines::registration::RegistrationResult registration_result;
    for (int i = 0; i < 3; ++i) {
        auto preliminary = ModifiedRegistrationRANSACBasedOnFeatureMatching(
            *source, *target, *source_fpfh, *target_fpfh, mutual_filter,
            1.5 * 0.12,
            registration::TransformationEstimationPointToPoint(false), 4,
            correspondence_checker,
            registration::RANSACConvergenceCriteria(MAX_REPEATS, CERTAINTY));
        registration_result = registration::RegistrationICP(
            *source, *target, 0.03, preliminary.transformation_);
        auto reverse_result = registration::EvaluateRegistration(
            *target, *source, 0.03,
            registration_result.transformation_.inverse());
        double min_quality =
            std::min(registration_result.fitness_, reverse_result.fitness_);
        if (min_quality > max_fitness) {
            max_fitness = min_quality;
            best_result = registration_result;
        }
        if (min_quality > 0.999) {
            break;
        }
    }
    VisualizeRegistration(*source, *target, best_result);
    return {max_fitness, best_result};
}

PoseEstimation::BestResult PoseEstimation::estimateTransformations(
    std::vector<std::pair<std::string, o3d::geometry::TriangleMesh>> &sources,
    std::vector<Ptr> &targets) {
    double best(0.3);
    BestResult best_result;
    ROS_INFO_STREAM("Target and Source size: " << targets.size() << ", "
                                               << sources.size());
    auto begin = std::chrono::system_clock::now();
    for (size_t t_idx = 0; t_idx < targets.size(); ++t_idx) {
        ROS_WARN_STREAM("Doing Target " << t_idx);
        const Ptr &potential_target = targets[t_idx];
        std::size_t n_points = potential_target->points_.size();
        for (size_t s_idx = 0; s_idx < sources.size(); ++s_idx) {
            ROS_WARN_STREAM("Doing Source " << s_idx);
            auto [name, mesh] = sources[s_idx];
            auto pcd = mesh.SamplePointsUniformly(n_points);
            // std::string baking("BakingVanilla");
            // if (name.find(baking) != std::string::npos) {
            // pcd->Scale(1.25, pcd->GetCenter());
            //}
            pcd->EstimateNormals();
            auto [fitness, result] = globalRegistration(pcd, potential_target);
            ROS_WARN_STREAM("Fitness of the result is:" << fitness);
            if (fitness > best) {
                best_result.source_idx = s_idx;
                best_result.source_name = name;
                best_result.target_idx = t_idx;
                best_result.result = result;
                best_result.source = pcd;
                best_result.target = potential_target;
                best = fitness;
            }
            if (fitness > 0.999) {
                return best_result;
            }
        }
    }
    auto end = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(end - begin);
    ROS_WARN_STREAM("The estimation took: " << diff.count());
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
    object_pose::positions positions;
    std_msgs::Header header;
    header.frame_id = "base_link";
    header.seq = callback_received;
    positions.poses.header = header;
    for (auto const &result : results) {
        ROS_WARN_STREAM("The transform of: " << result.source_name << " is\n"
                                             << result.result.transformation_);
        geometry_msgs::Pose pose;
        Eigen::Affine3d transform;
        transform = result.result.transformation_;
        tf::poseEigenToMsg(transform, pose);
        positions.objects.push_back(result.source_name);
        positions.poses.poses.push_back(pose);
    }
    publisher.publish(positions);
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
