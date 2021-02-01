#include <geometry_msgs/TransformStamped.h>
//#include <ros/ros.h>
//#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include "io_helper_functions.hpp"

// void poseCallback(const turtlesim::PoseConstPtr& msg) {
// static tf2_ros::TransformBroadcaster br;
// geometry_msgs::TransformStamped transformStamped;

// transformStamped.header.stamp = ros::Time::now();
// transformStamped.header.frame_id = "world";
// transformStamped.child_frame_id = turtle_name;
// transformStamped.transform.translation.x = msg->x;
// transformStamped.transform.translation.y = msg->y;
// transformStamped.transform.translation.z = 0.0;
// tf2::Quaternion q;
// q.setRPY(0, 0, msg->theta);
// transformStamped.transform.rotation.x = q.x();
// transformStamped.transform.rotation.y = q.y();
// transformStamped.transform.rotation.z = q.z();
// transformStamped.transform.rotation.w = q.w();

// br.sendTransform(transformStamped);
//}

int main(int argc, char** argv) {
    ros::init(argc, argv, "broadcast_transform");
    ros::NodeHandle node_handle;
    const vision::Paras paras = vision::get_paras(node_handle);
    const Eigen::Affine3d T_hand_eye =
        vision::read_hand_to_eye_transform(paras.transform_file);
    const std::string from("world"), to("forearm_link");
    cv::Mat depth_img, color_img;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    Eigen::Affine3d T_base_hand, T_base_camera, T_base_camera_inv;
    ros::Rate rate(50);
    geometry_msgs::TransformStamped trans;
    tf2_ros::TransformBroadcaster br;
    while (node_handle.ok()) {
        if (!vision::obtain_transform(from, to, tfBuffer, T_base_hand)) {
            continue;
        }
        T_base_camera = T_base_hand * T_hand_eye;
        T_base_camera_inv = T_base_camera.inverse();
        geometry_msgs::TransformStamped trans =
            tf2::eigenToTransform(T_base_camera_inv);
        trans.header.frame_id = "base_link";
        trans.child_frame_id = "camera_link";
        trans.header.stamp = ros::Time::now();
        br.sendTransform(trans);
        rate.sleep();
    }
    return 0;
}
