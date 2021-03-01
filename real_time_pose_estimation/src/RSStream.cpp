#include "RSStream.hpp"
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
RSStream::RSStream(const DetectionParameters& paras):
    align_to(RS2_STREAM_COLOR) {
    rs2::pipeline_profile profile = pipe.start();
}
    
void RSStream::extract_frames(const rs2::frameset& stream, cv::Mat& img) {
    rs2::frame color = stream.get_color_frame();
    static const int w = color.as<rs2::video_frame>().get_width();
    static const int h = color.as<rs2::video_frame>().get_height();
    img = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color.get_data(),
                        cv::Mat::AUTO_STEP);
}
bool RSStream::read(cv::Mat& img) {
        rs2::frameset data = pipe.wait_for_frames();  // Wait for next frame
        rs2::frameset aligned_set = align_to.process(data);
        extract_frames(aligned_set, img);
        return true;
}
