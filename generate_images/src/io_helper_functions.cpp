#include "../include/io_helper_functions.hpp"

#include <tf2_eigen/tf2_eigen.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

template <typename... Args>
std::string string_format(const std::string& format, Args... args) {
    int size = snprintf(nullptr, 0, format.c_str(), args...) +
               1;  // Extra space for '\0'
    if (size <= 0) {
        throw std::runtime_error("Error during formatting.");
    }
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(),
                       buf.get() + size - 1);  // We don't want the '\0' inside
}

std::string get_param(const ros::NodeHandle& node_handle,
                      const std::string& name, const std::string& identifier) {
    std::string result;
    if ((node_handle.getParam(name, result))) {
        std::string msg = "the " + identifier + " is at " + result;
        std::cout << msg << std::endl;
        ROS_DEBUG("%s", msg.c_str());
    } else {
        std::string failure =
            "Could not load " + identifier + " with name:\n" + name;
        ROS_DEBUG("%s", failure.c_str());
        ros::shutdown();
        throw std::ios_base::failure(failure);
    }
    return result;
}

cv::FileStorage read_open_cv_file(const std::string& path) {
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);
    return fs;
}

namespace vision {
void extract_frames(const rs2::frameset& stream, cv::Mat& depth_img,
                    cv::Mat& color_img) {
    rs2::frame depth = stream.get_depth_frame();
    rs2::frame color = stream.get_color_frame();
    static const int w = depth.as<rs2::video_frame>().get_width();
    static const int h = depth.as<rs2::video_frame>().get_height();
    depth_img = cv::Mat(cv::Size(w, h), CV_16UC1, (void*)depth.get_data(),
                        cv::Mat::AUTO_STEP);
    color_img = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color.get_data(),
                        cv::Mat::AUTO_STEP);
    // cv::cvtColor(r_rgb, color_img, cv::COLOR_RGB2BGR);
}

void write_intrinsics_to_file(const rs2::pipeline_profile& profile,
                              const std::string& root_dir) {
    rs2::video_stream_profile vsp(profile.get_stream(RS2_STREAM_COLOR));
    const rs2_intrinsics Kc = vsp.get_intrinsics();
    std::ofstream intrinsics_file;
    std::string location = root_dir + "camera_calibration.txt";
    intrinsics_file.open(location);
    if (!intrinsics_file.is_open()) {
        throw std::ios_base::failure("Could not open intrinics file");
    }
    float zero = 0.0;
    float one = 1.0;
    intrinsics_file << std::scientific;
    intrinsics_file << Kc.fx << " " << zero << " " << Kc.ppx << "\n";
    intrinsics_file << zero << " " << Kc.fy << " " << Kc.ppy << "\n";
    intrinsics_file << zero << " " << zero << " " << one << "\n";
    std::ofstream img_details;
    std::string img_location = root_dir + "img_details.txt";
    img_details.open(img_location);
    if (!img_details.is_open()) {
        throw std::ios_base::failure("Could not open intrinics file");
    }
    img_details << "Height: " << Kc.height << "\n";
    img_details << "Width: " << Kc.width << "\n";
}

void create_folders_if_neccessary(const std::string& root) {
    namespace fs = std::filesystem;
    if (!fs::is_directory(root) || !fs::exists(root)) {
        fs::create_directory(root);
    }
    std::string rgbd_dir = root + "/rgbd/";
    if (fs::exists(rgbd_dir)) {
        fs::remove_all(rgbd_dir);
    }
    if (!fs::is_directory(rgbd_dir) || !fs::exists(rgbd_dir)) {
        fs::create_directory(rgbd_dir);
    }
}

void write_frames_to_file(cv::Mat color, cv::Mat depth, const std::string& root,
                          size_t counter) {
    static const std::string rgbd_dir = root + "/rgbd/";
    std::string number = string_format("%04d", counter);
    const std::string depth_file = rgbd_dir + "frame-" + number + ".depth.png";
    const std::string col_file = rgbd_dir + "frame-" + number + ".color.png";
    cv::imwrite(depth_file, depth);
    cv::imwrite(col_file, color);
}

void write_transform_to_file(const Eigen::Affine3d& transform,
                             const std::string& root, size_t counter) {
    static const std::string rgbd_dir = root + "/rgbd/";
    const std::string number = string_format("%04d", counter);
    const std::string pose_loc = rgbd_dir + "frame-" + number + ".pose.txt";
    std::ofstream pose_file;
    pose_file << std::scientific;
    pose_file.open(pose_loc);
    if (!pose_file.is_open()) {
        throw std::ios_base::failure("Could not open pose file");
    }
    pose_file << transform.matrix() << "\n";
}

// This does not work! This is a fixed namespace, but it must be for each
// exetcuable!
Paras get_paras(const ros::NodeHandle& node_handle) {
    Paras paras;
    paras.root_dir =
        get_param(node_handle, "/BroadcastTransform/root_dir", "root director");
    paras.transform_file =
        get_param(node_handle, "/BroadcastTransform/transform_file",
                  "trasformation file");
    return paras;
}

Eigen::Matrix4d read_transform(const cv::FileStorage& fs,
                               const std::string& pos) {
    cv::Mat transformation;
    fs[pos] >> transformation;
    Eigen::Matrix4d tmp;
    cv::cv2eigen(transformation, tmp);
    return tmp;
}

const Eigen::Affine3d read_hand_to_eye_transform(const std::string& loc) {
    cv::FileStorage nodes = read_open_cv_file(loc);
    if (!nodes.isOpened()) {
        std::cerr << "Failed to open " << loc << std::endl;
    }
    const Eigen::Matrix4d mat = read_transform(nodes, "handToEyeTransform");
    Eigen::Affine3d trans(mat);
    std::cout << "The tranformation is:\n" << trans.matrix() << std::endl;
    return trans;
}

bool obtain_transform(std::string from, std::string to,
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

}  // namespace vision
