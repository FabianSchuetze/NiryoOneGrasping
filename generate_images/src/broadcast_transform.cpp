#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

#include "io_helper_functions.hpp"

void broadcast(const Eigen::Affine3d& transform) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped trans = tf2::eigenToTransform(transform);
    trans.header.frame_id = "base_link";
    trans.child_frame_id = "camera_link";
    trans.header.stamp = ros::Time::now();
    br.sendTransform(trans);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "broadcast_transform");
    ros::NodeHandle node_handle;
    const vision::Paras paras = vision::get_paras(node_handle);
    const Eigen::Affine3d T_hand_eye =
        vision::read_hand_to_eye_transform(paras.transform_file);
    const std::string from("world"), to("forearm_link");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    Eigen::Affine3d T_base_hand, T_base_camera;
    ros::Rate rate(50);
    while (node_handle.ok()) {
        if (!vision::obtain_transform(from, to, tfBuffer, T_base_hand)) {
            continue;
        }
        T_base_camera = T_base_hand * T_hand_eye;
        broadcast(T_base_camera);
        rate.sleep();
    }
    return 0;
}
