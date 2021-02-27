#include "ros_interaction.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
bool obtain_transform(const std::string& from, const std::string& to,
                      const tf2_ros::Buffer& buffer, Eigen::Affine3d& T_curr) {
    static geometry_msgs::TransformStamped trans;
    try {
        trans = buffer.lookupTransform(from, to, ros::Time(0));
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    T_curr = tf2::transformToEigen(trans);
    return true;
}

void broadcast(const Eigen::Affine3d& transform, const std::string& from,
        const std::string to) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped trans = tf2::eigenToTransform(transform);
    trans.header.frame_id = from;
    trans.child_frame_id = to;
    trans.header.stamp = ros::Time::now();
    br.sendTransform(trans);
}
