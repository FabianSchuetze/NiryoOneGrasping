#include "GPDInteraction.hpp"
#include <filesystem>
#include <open3d/Open3D.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
namespace fs = std::filesystem;
static constexpr std::size_t MAX_UINT(255);
static constexpr std::size_t N_SAMPLES(50000);
static constexpr float TEN_CENTIMER(0.1);

namespace gpd {
// TODO: Check if I can use this somewhere else
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

double calculateYaw(const geometry_msgs::Quaternion &quat) {
    double first = 2 * (quat.w * quat.z + quat.x * quat.y);
    double second = 1 - 2 * (quat.x * quat.x + quat.z * quat.z);
    double yaw = std::atan2(first, second);
    return yaw;
}

// TODO: Check if I can use this somewhere else?
std::tuple<double, double, double> RPY(const Eigen::Isometry3d &transform) {
    Eigen::Matrix3d tmp = transform.matrix().block(0, 0, 3, 3);
    Eigen::Quaternion<double> quat(tmp);
    tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
    tf::Matrix3x3 rotation(q);
    double roll(0.0), pitch(0.0), yaw(0.0);
    rotation.getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}

std::string shortName(const std::string &input_name,
                      std::string extension) { // NOLINT
    const std::string fn = fs::path(input_name).filename();
    std::string delimiter = "_";
    std::string token = fn.substr(0, fn.find(delimiter));
    return token + extension;
}

geometry_msgs::TransformStamped rosTransform(const Eigen::Isometry3d &transform,
                                             const std::string &name,
                                             std::string extension) { // NOLINT
    auto tmp = tf2::eigenToTransform(transform);
    tmp.header.frame_id = "base_link";
    tmp.child_frame_id = shortName(name, extension);
    return tmp;
}

geometry_msgs::Pose generateGraspPose(const Hand &hand) {
    geometry_msgs::Pose grasp_pose;
    grasp_pose.position.x = hand.x;
    grasp_pose.position.y = hand.y;
    grasp_pose.position.z = hand.z;
    tf2::Quaternion q;
    q.setRPY(0, hand.pitch, hand.yaw);
    grasp_pose.orientation = tf2::toMsg(q);
    return grasp_pose;
}

Eigen::Isometry3d GPDInteraction::generateTransformation(const Hand &hand) {
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose grasp_pose;
    transformStamped.transform.translation.x = hand.x;
    transformStamped.transform.translation.y = hand.y;
    transformStamped.transform.translation.z = hand.z;
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
    pub_ = n_.advertise<geometry_msgs::PoseArray>(publication, 1, true);
    publish_pointcloud_ =
        n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/cloud_pcd", 1);
}

Eigen::Isometry3d
GPDInteraction::convertHandToTransform(const gpd_ros::GraspConfig &grasp) {
    Eigen::Isometry3d trans(Eigen::Matrix4d::Identity(4, 4));
    // Eigen::Matrix4d frame = Eigen::MatrixXd::Identity(4, 4);
    Eigen::Vector3d approach, binormal, axis, position;
    tf2::fromMsg(grasp.approach, approach);
    tf2::fromMsg(grasp.binormal, binormal);
    tf2::fromMsg(grasp.axis, axis);
    tf2::fromMsg(grasp.position, position);
    trans.matrix().block<3, 1>(0, 0) = approach;
    trans.matrix().block<3, 1>(0, 1) = binormal;
    trans.matrix().block<3, 1>(0, 2) = axis;
    trans.matrix().block<3, 1>(0, 3) = position;
    // trans.matrix() = frame;
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
    if (!open3d::io::ReadTriangleMesh(name, mesh)) {
        ROS_ERROR_STREAM("Could not read mesh file " << name);
        throw std::runtime_error("Couldd not read mesh file");
    }
    auto cloud = mesh.SamplePointsUniformly(N_SAMPLES)->points_;
    // Eigen::Vector3d center = mesh.GetCenter();
    PointCloud::Ptr pcl_cloud(new PointCloud);
    // std::transform(cloud.begin(), cloud.end(),
    // std::back_inserter(*pcl_cloud),
    //[](auto x) { return toPointXYZRGB(x); });
    // does this work?
    for (const auto &o3d_point : cloud) {
        auto point = toPointXYZRGB(o3d_point);
        pcl_cloud->push_back(toPointXYZRGB(o3d_point));
    }
    pcl_cloud->height = 1;
    pcl_cloud->width = cloud.size();
    std::string fn = fs::path(name).filename();
    pcl_cloud->header.frame_id = fn;
    publish_pointcloud_.publish(pcl_cloud);
    // return center;
}

int GPDInteraction::filterPossibleTransforms(const Eigen::Isometry3d &object) {
    static Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    move.matrix()(0, 3) = TEN_CENTIMER;
    for (std::size_t i = 0; i < possible_transforms.size(); ++i) {
        auto grasp_frame = generateHand(object, i);
        auto finger_frame = grasp_frame * move;
        ROS_DEBUG_STREAM(
            "Niryo's finger height is above: " << finger_frame(2, 3));
        if (finger_frame(2, 3) > 0) { // finger must be above zero
            return i;
        }
    }
    return -1;
}

Eigen::Isometry3d GPDInteraction::generateHand(const Eigen::Isometry3d &object,
                                               int idx) {
    const auto grasp_frame = object * possible_transforms[idx];
    auto [roll_hand, pitch_hand, yaw_hand] = RPY(grasp_frame);
    Hand hand{grasp_frame(0, 3), grasp_frame(1, 3), grasp_frame(2, 3),
              pitch_hand, yaw_hand};
    auto res = generateTransformation(hand);
    res.matrix()(2, 3) = grasp_frame(2, 3);
    return res;
}

Eigen::Isometry3d
GPDInteraction::cleanObjectPose(const geometry_msgs::Pose &raw_pose) {
    Hand hand{raw_pose.position.x, raw_pose.position.y, 0, 0,
              calculateYaw(raw_pose.orientation)};
    Eigen::Isometry3d object_frame = generateTransformation(hand);
    return object_frame;
}

geometry_msgs::Pose
GPDInteraction::cleanGraspPose(const Eigen::Isometry3d &grasp_frame) {
    auto [roll_hand, pitch_hand, yaw_hand] = RPY(grasp_frame);
    Hand finalHand{grasp_frame(0, 3), grasp_frame(1, 3), grasp_frame(2, 3),
                   pitch_hand, yaw_hand};
    auto grasp_pose = generateGraspPose(finalHand);
    return grasp_pose;
}

void GPDInteraction::callback_object_pose(const object_pose::positions &msg) {
    grasp_pose_received = false;
    ROS_DEBUG_STREAM("Inside the object pose callback");
    const auto now = ros::Time::now();
    std::vector<geometry_msgs::TransformStamped> transforms;
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = msg.poses.header.frame_id;
    poses.header.stamp = now;
    for (std::size_t i = 0; i < msg.objects.size(); ++i) {
        grasp_pose_received = false;
        // geometry_msgs::Pose object_pose = msg.poses.poses[i];
        const std::string &name = msg.objects[i];
        // Hand hand{object_pose.position.x, object_pose.position.y, 0, 0,
        // calculateYaw(object_pose.orientation)};
        Eigen::Isometry3d object_frame = cleanObjectPose(msg.poses.poses[i]);
        // TODO Publish the transforms in hte object pose node
        // transforms.push_back(rosTransform(object_frame, name, ""));
        publishPointCloud(name);
        while ((!grasp_pose_received) and ros::ok()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        ROS_DEBUG_STREAM("Recived grasps of " << name);
        grasp_pose_received = false;
        int res = filterPossibleTransforms(object_frame);
        if (res < 0) {
            ROS_WARN_STREAM("Did not desocver a valid hand");
            continue;
        }
        // ROS_WARN_STREAM("Picked number " << res);
        Eigen::Isometry3d grasp_frame = generateHand(object_frame, res);
        // std::string grasp_name = "grasp_" + std::to_string(res);
        transforms.push_back(rosTransform(grasp_frame, name, "grasp"));
        auto grasp_pose = cleanGraspPose(grasp_frame);
        // auto [roll_hand, pitch_hand, yaw_hand] = RPY(grasp_frame);
        // Hand finalHand{grasp_frame(0, 3), grasp_frame(1, 3), grasp_frame(2,
        // 3), pitch_hand, yaw_hand};
        // auto grasp_pose = generateGraspPose(finalHand);
        poses.poses.push_back(grasp_pose);
    }
    ROS_WARN_STREAM("Transfroms size: " << transforms.size());
    pub_.publish(poses);
    br.sendTransform(transforms);
}
} // namespace gpd
