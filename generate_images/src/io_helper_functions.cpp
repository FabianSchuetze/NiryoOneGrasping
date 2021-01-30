#include "../include/io_helper_functions.hpp"

#include <fstream>

#include "opencv2/imgproc.hpp"
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
                              const std::string& location) {
    rs2::video_stream_profile vsp(profile.get_stream(RS2_STREAM_COLOR));
    const rs2_intrinsics Kc = vsp.get_intrinsics();
    std::ofstream intrinsics_file;
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
}
}  // namespace vision
