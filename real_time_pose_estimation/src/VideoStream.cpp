#include "VideoStream.hpp"
#include <iostream>

VideoStream::VideoStream(const DetectionParameters& paras) {
    cap.open(paras.video_read_path);  // open a recorded video
    if (!cap.isOpened())              // check if we succeeded
    {
        std::cout << "Could not open the camera device" << std::endl;
        throw std::runtime_error("");
    }
}
bool VideoStream::read(cv::Mat& img) {
    bool result = cap.read(img);
    return result;
}
