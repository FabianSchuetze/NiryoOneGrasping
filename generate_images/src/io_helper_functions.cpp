#include "../include/io_helper_functions.hpp"

#include <filesystem>
#include <fstream>
namespace fs = std::filesystem;

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

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

namespace vision {
void extract_frames(const rs2::frameset& stream, cv::Mat& depth_img,
                    cv::Mat& color_img) {
    rs2::frame depth = stream.get_depth_frame();
    rs2::frame color = stream.get_color_frame();
    static const int w = depth.as<rs2::video_frame>().get_width();
    static const int h = depth.as<rs2::video_frame>().get_height();
    depth_img = cv::Mat(cv::Size(w, h), CV_16UC1, (void*)depth.get_data(),
                        cv::Mat::AUTO_STEP);
    cv::Mat r_rgb(cv::Size(w, h), CV_8UC3, (void*)color.get_data(),
                  cv::Mat::AUTO_STEP);
    cv::cvtColor(r_rgb, color_img, cv::COLOR_RGB2BGR);
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
    img_details << "Height: " << Kc.height <<"\n";
    img_details << "Width: " << Kc.width <<"\n";
}

void create_folders_if_neccessary(const std::string& root) {
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
}  // namespace vision
