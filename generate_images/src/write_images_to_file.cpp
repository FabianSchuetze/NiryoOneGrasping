#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "io_helper_functions.hpp"
//#include <opencv2/imgcodecs.hpp>
//#include "opencv2/imgproc.hpp"

int main() {
    rs2::pipeline pipe;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline_profile profile = pipe.start();
    const std::string color_file("color.png"), depth_file("depth.png");
    const std::string window_name = "Display Image";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::Mat depth_img, color_img;
    std::string location("calibration.txt");
    vision::write_intrinsics_to_file(profile, location);
    while (cv::waitKey(1) < 0 &&
           getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0) {
        rs2::frameset data = pipe.wait_for_frames();  // Wait for next frame
        rs2::frameset aligned_set = align_to.process(data);
        vision::extract_frames(aligned_set, depth_img, color_img);
        cv::imwrite(depth_file, depth_img);
        cv::imwrite(color_file, color_img);
        cv::imshow(window_name, color_img);
    }
    return EXIT_SUCCESS;
}
