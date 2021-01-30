#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
// not sure if all of these includes are needed:
//#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

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

void write_intrinsics_to_file(const rs2::pipeline_profile& profile) {
    rs2::video_stream_profile vsp(profile.get_stream(RS2_STREAM_COLOR));
    const rs2_intrinsics intrincis = vsp.get_intrinsics();
}

int main() {
    rs2::pipeline pipe;
    rs2::align align_to(RS2_STREAM_COLOR);
    pipe.start();
    const std::string color_file("color.png"), depth_file("depth.png");
    const std::string window_name = "Display Image";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::Mat depth_img, color_img;
    while (cv::waitKey(1) < 0 &&
           getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0) {
        rs2::frameset data = pipe.wait_for_frames();  // Wait for next frame
        rs2::frameset aligned_set = align_to.process(data);
        extract_frames(aligned_set, depth_img, color_img);
        cv::imwrite(depth_file, depth_img);
        cv::imwrite(color_file, color_img);
        cv::imshow(window_name, color_img);
    }
    return EXIT_SUCCESS;
}
