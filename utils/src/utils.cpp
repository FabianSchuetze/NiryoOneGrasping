#include "utils/utils.hpp"
#include <filesystem>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace utils {

DOF::DOF(double x_, double y_, double z_, double roll_, double pitch_,
         double yaw_)
    : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_){};

geometry_msgs::TransformStamped DOF::transformStamped() const {
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose grasp_pose;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    return transformStamped;
}

geometry_msgs::Pose DOF::pose() const {
    geometry_msgs::Pose grasp_pose;
    grasp_pose.position.x = x;
    grasp_pose.position.y = y;
    grasp_pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    grasp_pose.orientation = tf2::toMsg(q);
    return grasp_pose;
}

std::tuple<double, double, double> RPY(const tf::Quaternion &quat) {
    tf::Matrix3x3 rotation(quat);
    double roll(0.0), pitch(0.0), yaw(0.0);
    rotation.getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}

std::tuple<double, double, double> RPY(const Eigen::Isometry3d &transform) {
    Eigen::Matrix3d tmp = transform.linear();
    Eigen::Quaternion<double> quat(tmp);
    tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
    return RPY(q);
    // tf::Matrix3x3 rotation(q);
    // double roll(0.0), pitch(0.0), yaw(0.0);
    // rotation.getRPY(roll, pitch, yaw);
    // return {roll, pitch, yaw};
}
std::tuple<double, double, double> RPY(const geometry_msgs::Quaternion &quat) {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    return RPY(q);
    // tf::Matrix3x3 rotation(q);
    // double roll(0.0), pitch(0.0), yaw(0.0);
    // rotation.getRPY(roll, pitch, yaw);
    // return {roll, pitch, yaw};
}
std::string shortName(const std::string &input_name,
                      const std::string &extension) {
    const std::string fn = std::filesystem::path(input_name).filename();
    std::string delimiter = "_";
    std::string token = fn.substr(0, fn.find(delimiter));
    // ROS_WARN_STREAM("Incoming: " << input_name << ", "
    //<< "token: " << token);
    return token + extension;
}
} // namespace utils
