#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <opencv2/core.hpp>
#include <ros/ros.h>

#include "io_helper_functions.hpp"

std::string get_paras(const ros::NodeHandle& node_handle) {
    std::string root_dir;
    if ((node_handle.getParam("/WriteImagesToFile/root_dir", root_dir))) {
        std::cout << "the root directory is at: " << root_dir << std::endl;
        ROS_DEBUG("the file location is: %s", root_dir.c_str());
    } else {
        ROS_DEBUG("that did not work: %s", root_dir.c_str());
        ros::shutdown();
        throw std::ios_base::failure("Error during formatting.");
    }
    return root_dir;
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "write_images_to_file");
    ros::NodeHandle node_handle;
    //ros::AsyncSpinner spinner(1);
    std::string root_dir = get_paras(node_handle);
    rs2::pipeline pipe;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline_profile profile = pipe.start();
    const std::string color_file("color.png"), depth_file("depth.png");
    //const std::string window_name = "Display Image";
    cv::Mat depth_img, color_img;
    vision::create_folders_if_neccessary(root_dir);
    vision::write_intrinsics_to_file(profile, root_dir);
    for (int i = 0; i < 30; ++i) {
        rs2::frameset data = pipe.wait_for_frames();
        rs2::frameset aligned_set = align_to.process(data);
    } // needs to warm up
    size_t counter(0);
    ros::Rate rate(5);
    while (ros::ok()) {
        rs2::frameset data = pipe.wait_for_frames();  // Wait for next frame
        rs2::frameset aligned_set = align_to.process(data);
        vision::extract_frames(aligned_set, depth_img, color_img);
        vision::write_frames_to_file(color_img, depth_img, root_dir, counter);
        ++counter;
        rate.sleep();
    }
    return EXIT_SUCCESS;
}
