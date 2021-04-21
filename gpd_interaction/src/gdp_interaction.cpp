#include "GPDInteraction.hpp"
#include <filesystem>
#include <open3d/Open3D.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
namespace fs = std::filesystem;
constexpr double PI(3.14);
constexpr double HEIGHT_BAKING(0.08);
static constexpr std::size_t MAX_UINT(255);
static constexpr std::size_t N_SAMPLES(50000);
static constexpr std::size_t BEST_SAMPLES(10);

namespace gpd {
pcl::PointXYZRGB inline toPointXYZRGB(const Eigen::Vector3d &point) {
    pcl::PointXYZRGB pcl_point;
    pcl_point.x = static_cast<float>(point(0));
    pcl_point.y = static_cast<float>(point(1));
    pcl_point.z = static_cast<float>(point(2));
    pcl_point.r = static_cast<uint8_t>(0);
    pcl_point.g = static_cast<uint8_t>(0);
    pcl_point.b = static_cast<uint8_t>(MAX_UINT);
    return pcl_point;
}
double calculateRoll(const Eigen::Quaterniond &quat) {

    double first = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
    double second = 1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z());
    double roll = std::atan2(first, second);
    return roll;
}
double calculateYaw(const Eigen::Quaterniond &quat) {

    double first = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
    double second = 1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z());
    double yaw = std::atan2(first, second);
    return yaw;
}

double calculateYaw(const geometry_msgs::Quaternion &quat) {
    double first = 2 * (quat.w * quat.z + quat.x * quat.y);
    double second = 1 - 2 * (quat.x * quat.x + quat.z * quat.z);
    double yaw = std::atan2(first, second);
    return yaw;
}

double calculatePitch(const Eigen::Quaterniond &quat) {

    double first = 2 * (quat.w() * quat.y() - quat.y() * quat.x());
    double pitch = std::asin(first);
    return pitch;
}

std::tuple<double, double, double> RPY(const Eigen::Isometry3d &transform) {
    Eigen::Matrix3d tmp = transform.matrix().block(0, 0, 3, 3);
    Eigen::Quaternion<double> quat(tmp);
    double roll = calculateRoll(quat);
    double pitch = calculatePitch(quat);
    double yaw = calculateYaw(quat);
    return {roll, pitch, yaw};
}

std::string shortName(const std::string &input_name, std::string extension) {
    const std::string fn = fs::path(input_name).filename();
    std::string delimiter = "_";
    std::string token = fn.substr(0, fn.find(delimiter));
    return token + extension;
}

geometry_msgs::TransformStamped rosTransform(const Eigen::Isometry3d &transform,
                                             const std::string &name,
                                             std::string extension) {
    auto tmp = tf2::eigenToTransform(transform);
    tmp.header.frame_id = "base_link";
    tmp.child_frame_id = shortName(name, extension);
    return tmp;
}

Eigen::Isometry3d GPDInteraction::generateTransformation(const Hand &hand) {
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose grasp_pose;
    transformStamped.transform.translation.x = hand.x;
    transformStamped.transform.translation.y = hand.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, hand.pitch, hand.yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    Eigen::Isometry3d transform = tf2::transformToEigen(transformStamped);
    return transform;
}

GPDInteraction::GPDInteraction(ros::NodeHandle &n_,
                               const std::string &publication)
    : grasp_pose_received(false) {
    // ROS_WARN_STREAM("The publication is" << publication);
    pub_ = n_.advertise<geometry_msgs::PoseArray>(publication, 1, true);
    publish_pointcloud_ =
        n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/cloud_pcd", 1);
}

Eigen::Isometry3d
GPDInteraction::convertHandToTransform(const gpd_ros::GraspConfig &grasp) {
    Eigen::Isometry3d trans;
    Eigen::Matrix4d frame = Eigen::MatrixXd::Identity(4, 4);
    Eigen::Vector3d approach, binormal, axis, position;
    tf2::fromMsg(grasp.approach, approach);
    tf2::fromMsg(grasp.binormal, binormal);
    tf2::fromMsg(grasp.axis, axis);
    tf2::fromMsg(grasp.position, position);
    frame.block<3, 1>(0, 0) = approach;
    frame.block<3, 1>(0, 1) = binormal;
    frame.block<3, 1>(0, 2) = axis;
    frame.block<3, 1>(0, 3) = position;
    trans.matrix() = frame;
    return trans;
}

void GPDInteraction::callback_grasp_pose(const gpd_ros::GraspConfigList &msg) {
    possible_transforms.clear();
    for (auto const &grasp : msg.grasps) {
        auto possible_transform = convertHandToTransform(grasp);
        possible_transforms.push_back(possible_transform);
    }
    grasp_pose_received = true;
}

void GPDInteraction::publishPointCloud(const std::string &name) {
    open3d::geometry::TriangleMesh mesh;
    bool success = open3d::io::ReadTriangleMesh(name, mesh);
    if (!success) {
        ROS_ERROR_STREAM("Could not read mesh file " << name);
        throw std::runtime_error("Couldd not read mesh file");
    }
    auto cloud = mesh.SamplePointsUniformly(N_SAMPLES);
    PointCloud::Ptr pcl_cloud(new PointCloud);
    for (const auto &o3d_point : cloud->points_) {
        auto point = toPointXYZRGB(o3d_point);
        pcl_cloud->push_back(point);
    }
    pcl_cloud->height = 1;
    pcl_cloud->width = cloud->points_.size();
    std::string fn = fs::path(name).filename();
    pcl_cloud->header.frame_id = fn;
    publish_pointcloud_.publish(pcl_cloud);
}

int GPDInteraction::filterPossibleTransforms(const Eigen::Isometry3d &object) {
    int best_hand(-1);
    auto [roll_object, pitch_object, yaw_object] = RPY(object);
    double error = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < BEST_SAMPLES;
         ++i) { // check the best 10 results {
        const auto &possible_transform = possible_transforms[i];
        const auto hand = object * possible_transform;
        auto [roll_hand, pitch_hand, yaw_hand] = RPY(hand);
        if (pitch_hand > 1.5 || pitch_hand < -0.5) {
            continue;
        }
        double curr_error = std::abs(yaw_object - yaw_hand);
        if (curr_error < error) {
            best_hand = i;
            error = curr_error;
        }
    }
    return best_hand;
}

Eigen::Isometry3d GPDInteraction::generateHand(const Eigen::Isometry3d &object,
                                               int idx) {
    const auto grasp_frame = object * possible_transforms[idx];
    auto [roll_hand, pitch_hand, yaw_hand] = RPY(grasp_frame);
    Hand hand{grasp_frame(0, 3), grasp_frame(1, 3), pitch_hand, yaw_hand};
    auto res = generateTransformation(hand);
    return res;
}

void GPDInteraction::callback_object_pose(const object_pose::positions &msg) {
    grasp_pose_received = false;
    ROS_WARN_STREAM("Inside the object pose callback");
    // const auto now = ros::Time::now();
    std::vector<geometry_msgs::TransformStamped> transforms;
    // geometry_msgs::PoseArray poses;
    // poses.header.frame_id = msg.poses.header.frame_id;
    // poses.header.stamp = now;
    for (std::size_t i = 0; i < msg.objects.size(); ++i) {
        grasp_pose_received = false;
        geometry_msgs::Pose pose = msg.poses.poses[i];
        const std::string &name = msg.objects[i];
        Hand hand{pose.position.x, pose.position.y, 0,
                  calculateYaw(pose.orientation)};
        Eigen::Isometry3d object_frame = generateTransformation(hand);
        transforms.push_back(rosTransform(object_frame, name, ""));
        publishPointCloud(name);
        while ((!grasp_pose_received) and ros::ok()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        grasp_pose_received = false;
        int res = filterPossibleTransforms(object_frame);
        if (res < 0) {
            ROS_WARN_STREAM("Did not desocver a valid hand");
            continue;
        }
        Eigen::Isometry3d grasp_frame = generateHand(object_frame, res);
        Eigen::Isometry3d tmp = object_frame * grasp_frame;
        transforms.push_back(rosTransform(tmp, name, "_grasp"));
    }
    ROS_WARN_STREAM("Transfroms size: " << transforms.size());
    // pub_.publish(poses);
    br.sendTransform(transforms);
}
} // namespace gpd
