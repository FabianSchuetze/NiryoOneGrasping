#include "integrate.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <sstream>
using o3d::geometry::Image;
namespace fs = std::filesystem;
namespace integration {
constexpr int TOMM = 1000;
constexpr uint HEIGHT = 480;
constexpr uint WIDTH = 640;

std::string Integration::return_current_time_and_date() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M");
    return ss.str();
}

Integration::Paths Integration::open_folder(const std::string &dest) {
    std::string current_date = return_current_time_and_date();
    std::filesystem::path second_root(current_date);
    // TODO: Turn this into a parameter
    std::filesystem::path root(dest);
    // auto root("/root/generate_samples/src/generate_samples/pick_place/data");
    Paths paths;
    paths.depth = root / second_root / fs::path("depth");
    paths.color = root / second_root / fs::path("color");
    paths.transform = root / second_root / fs::path("transform");
    fs::create_directories(paths.color);
    fs::create_directories(paths.depth);
    fs::create_directories(paths.transform);
    return paths;
}

void Integration::save_img(const std::shared_ptr<o3d::geometry::Image> &img,
                           const fs::path &path, int iter) {
    fs::path name;
    if (iter < 10) {
        name = "00" + std::to_string(iter) + ".png";
    } else if (iter < 100) {
        name = "0" + std::to_string(iter) + ".png";
    } else {
        name = std::to_string(iter) + ".png";
    }
    const fs::path fn = path / name;
    std::cout << "location: " << fn << std::endl;
    bool success = o3d::io::WriteImage(fn, *img);
    if (!success) {
        std::stringstream ss;
        ss << "Could not save image at " << fn;
        ROS_WARN_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
}
std::shared_ptr<Image>
Integration::decipherDepth(const PointCloud::Ptr &cloud) {
    if (cloud->height != HEIGHT) {
        throw std::runtime_error("Has different height");
    }
    if (cloud->width != WIDTH) {
        throw std::runtime_error("Has different width");
    }
    cv::Mat img = cv::Mat(HEIGHT, WIDTH, CV_16UC1);
    for (size_t row = 0; row < HEIGHT; ++row) {
        for (size_t column = 0; column < WIDTH; ++column) {
            const auto &pt = cloud->at(column, row);
            auto val = static_cast<ushort>(std::round(pt.z * TOMM)); // NOLINT
            img.at<ushort>(row, column) = val;
        }
    }
    auto o3d_img = std::make_shared<Image>();
    o3d_img->Prepare(img.cols, img.rows, 1, sizeof(ushort));
    std::memcpy(o3d_img->data_.data(), img.data, o3d_img->data_.size());
    return o3d_img;
}
std::shared_ptr<Image>
Integration::decipherImage(const PointCloud::Ptr &cloud) {
    sensor_msgs::Image ros_img;
    pcl::toROSMsg(*cloud, ros_img);
    cv_bridge::CvImagePtr img =
        cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    if (img->image.empty()) {
        std::cout << "Could not read the image: " << std::endl;
    }
    auto o3d_img = std::make_shared<Image>();
    o3d_img->Prepare(img->image.cols, img->image.rows, 3, sizeof(uint8_t));
    std::memcpy(o3d_img->data_.data(), img->image.data, o3d_img->data_.size());
    return o3d_img;
}

void Integration::callback(const PointCloud::Ptr &cloud) {
    auto color = decipherImage(cloud);
    auto depth = decipherDepth(cloud);
    colors.push_back(color);
    depths.push_back(depth);
    save_img(color, paths.color, i);
    save_img(depth, paths.depth, i);
    ++i;
}
} // namespace integration
