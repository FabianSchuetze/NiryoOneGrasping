#include "RSStream.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
namespace fs = std::filesystem;

std::string return_current_time_and_date() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M");
    return ss.str();
}
void RSStream::open_folder(const DetectionParameters& paras) {
    std::string current_date = return_current_time_and_date();
    fs::path second_root(current_date);
    fs::path root = paras.save_storage;
    folder = root / second_root;
    fs::create_directory(folder);
}

RSStream::RSStream(const DetectionParameters& paras)
    : align_to(RS2_STREAM_COLOR), iter(0) {
    pipe.start();
    for (int i = 0; i < 30; ++i) {
        pipe.wait_for_frames();
    }
    open_folder(paras);
}

void RSStream::extract_frames(const rs2::frameset& stream, cv::Mat& img) {
    rs2::frame color = stream.get_color_frame();
    static const int w = color.as<rs2::video_frame>().get_width();
    static const int h = color.as<rs2::video_frame>().get_height();
    img = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color.get_data(),
                  cv::Mat::AUTO_STEP);
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    fs::path name;
    if (iter < 10) {
        name = "00" + std::to_string(iter) + ".png";
    } else if (iter < 100) {
        name = "0" + std::to_string(iter) + ".png";
    } else {
        name = std::to_string(iter) + ".png";
    }
    ++iter;
    const fs::path fn = folder / name;
    bool success = cv::imwrite(fn, img);
    if (!success) {
        throw std::runtime_error("Could not save in folder");
    }
}
bool RSStream::read(cv::Mat& img) {
    rs2::frameset data = pipe.wait_for_frames();  // Wait for next frame
    rs2::frameset aligned_set = align_to.process(data);
    extract_frames(aligned_set, img);
    return true;
}
