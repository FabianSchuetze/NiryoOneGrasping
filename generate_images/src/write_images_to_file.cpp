#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <opencv2/core.hpp>

#include "io_helper_functions.hpp"

void warmup_camera(const rs2::pipeline& pipe, const rs2::align& align_to) {
    for (int i = 0; i < 30; ++i) {
        rs2::frameset data = pipe.wait_for_frames();
        rs2::frameset aligned_set = align_to.process(data);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "write_images_to_file");
    ros::NodeHandle node_handle;
    rs2::pipeline pipe;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline_profile profile = pipe.start();
    const vision::Paras paras = vision::get_paras(node_handle);
    vision::create_folders_if_neccessary(paras.root_dir);
    vision::write_intrinsics_to_file(profile, paras.root_dir);
    const Eigen::Affine3d T_hand_eye =
        vision::read_hand_to_eye_transform(paras.transform_file);
    size_t counter(0);
    const std::string from("world"), to("tool_link");
    cv::Mat depth_img, color_img;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    Eigen::Affine3d T_base_hand, T_base_camera;
    ros::Rate rate(1);
    while (ros::ok()) {
        rs2::frameset data = pipe.wait_for_frames();  // Wait for next frame
        if (!vision::obtain_transform(from, to, tfBuffer, T_base_hand)) {
            continue;
        }
        T_base_camera = T_base_hand * T_hand_eye;
        rs2::frameset aligned_set = align_to.process(data);
        vision::extract_frames(aligned_set, depth_img, color_img);
        vision::write_frames_to_file(color_img, depth_img, paras.root_dir,
                                     counter);
        vision::write_transform_to_file(T_base_camera, paras.root_dir, counter);
        ++counter;
        rate.sleep();
    }
    return EXIT_SUCCESS;
}
