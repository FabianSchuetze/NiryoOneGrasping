#include "grasp_pose_broadcaster.hpp"
#include <open3d/Open3D.h>
#include <tf2_eigen/tf2_eigen.h>
constexpr std::size_t QUEUE(10);
constexpr double BEND_ARM(1.5);
constexpr double HAND_LENGTH(0.08);
namespace GraspingFunction {
utils::DOF
centroidGraspPose(const geometry_msgs::TransformStamped &ros_transform,
                  const std::string &name, double yaw) {
    Eigen::Isometry3d transform = tf2::transformToEigen(ros_transform);
    ROS_WARN_STREAM("The incoming transform is:\n " << transform.matrix());
    auto [r, p, yw] = utils::RPY(transform);
    ROS_WARN_STREAM("The r,p, y: " << r << ", " << p << ", " << yw);
    double x = transform(0, 3);
    double y = transform(1, 3);
    double z = transform(2, 3) + HAND_LENGTH;
    utils::DOF dof(x, y, z, 0, BEND_ARM, yaw);
    return dof;
}

utils::DOF
generateGraspPose(const geometry_msgs::TransformStamped &ros_transform,
                  const std::string &name, double yaw) {
    open3d::geometry::TriangleMesh mesh;
    if (!open3d::io::ReadTriangleMesh(name, mesh)) {
        ROS_ERROR_STREAM("Could not read mesh file " << name);
        throw std::runtime_error("Couldd not read mesh file");
    }
    Eigen::Isometry3d transform = tf2::transformToEigen(ros_transform);
    ROS_WARN_STREAM("The incoming transform is:\n " << transform.matrix());
    auto [r, p, yw] = utils::RPY(transform);
    ROS_WARN_STREAM("The r,p, y: " << r << ", " << p << ", " << yw);
    mesh.Transform(transform.matrix());
    const Eigen::Vector3d center = mesh.GetCenter();
    const Eigen::Vector3d bound = mesh.GetMaxBound();
    double x = center(0);
    double y = center(1);
    double z = bound(2) + 0.05;
    utils::DOF dof(x, y, z, 0, BEND_ARM, yaw);
    return dof;
}
} // namespace GraspingFunction

GraspPoseBroadcaster::GraspPoseBroadcaster(ros::NodeHandle &n_,
                                           const std::string &publication,
                                           const std::string &type)
    : current_iteration(0) {
    pub_ = n_.advertise<geometry_msgs::PoseArray>(publication, 1, true);
    if ((type == "/centroids") or (type == "/visual")) {
        grasping_func = &GraspingFunction::centroidGraspPose;
    } else if (type == "/geometric") {
        grasping_func = &GraspingFunction::generateGraspPose;
    } else {
        ROS_ERROR_STREAM("type must be /centroids | /geometric, got: " << type);
        throw std::runtime_error("type must be Clustering or Geometric");
    }
    ROS_WARN_STREAM("Completed the constructor");
}

void GraspPoseBroadcaster::callback(const object_pose::positions &msg) {
    const auto now = ros::Time::now();
    std::vector<geometry_msgs::TransformStamped> transforms;
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = msg.poses.header.frame_id;
    poses.header.stamp = now;
    if (msg.poses.header.seq < current_iteration) {
        ROS_ERROR_STREAM("Got pose with seq id: " << msg.poses.header.seq <<
                ", but have already processed " << current_iteration <<
                ", sequence. Abort!");
        return;
    }
    ROS_WARN_STREAM("Inside the callback");
    for (std::size_t i = 0; i < msg.objects.size(); ++i) {
        geometry_msgs::Pose pose = msg.poses.poses[i];
        const std::string &name = msg.objects[i];
        auto [roll, pitch, yaw] = utils::RPY(pose.orientation);
        // TODO: The positive z axis also differs fro the other program!
        utils::DOF dof(pose.position.x, pose.position.y, pose.position.z, 0, 0,
                       yaw);
        auto transformStamped = dof.transformStamped();
        utils::DOF dof_out = grasping_func(transformStamped, name, yaw);
        geometry_msgs::TransformStamped tf_out = dof_out.transformStamped();
        geometry_msgs::Pose grasp_pose = dof_out.pose();
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = msg.poses.header.frame_id;
        tf_out.header.frame_id = msg.poses.header.frame_id;
        std::string sho = utils::shortName(name, "_in");
        std::string sho_out = utils::shortName(name, "_grasp");
        transformStamped.child_frame_id = sho;
        tf_out.child_frame_id = sho_out;
        ROS_WARN_STREAM("Grasp pose for "
                        << transformStamped.child_frame_id
                        << " is\n: x, y, z, yaw: " << grasp_pose.position.x
                        << ", " << grasp_pose.position.y << ", "
                        << grasp_pose.position.z << ", " << yaw);
        poses.poses.push_back(grasp_pose);
        transforms.push_back(transformStamped);
        transforms.push_back(tf_out);
    }
    pub_.publish(poses);
    br.sendTransform(transforms);
    ++current_iteration;
}
