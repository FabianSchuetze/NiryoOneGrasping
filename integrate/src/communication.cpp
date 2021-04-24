#include "integrate.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <sstream>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using o3d::geometry::Image;
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
namespace fs = std::filesystem;
namespace integration {
static constexpr int TOMM = 1000;
static constexpr uint HEIGHT = 480;
static constexpr uint WIDTH = 640;
static constexpr std::size_t MAX_UINT(255);
static constexpr std::size_t RATE(10);

std::string return_current_time_and_date() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M");
    return ss.str();
}

fs::path generatePath(const fs::path &root, int iter,
                      std::string ending) { // NOLINT
    fs::path name;
    if (iter < 10) {
        name = "00" + std::to_string(iter) + ending;
    } else if (iter < 100) {
        name = "0" + std::to_string(iter) + ending;
    } else {
        name = std::to_string(iter) + ending;
    }
    fs::path fn = root / name;
    return fn;
}

Integration::Paths Integration::open_folder(const std::string &dest) {
    std::string current_date = return_current_time_and_date();
    std::filesystem::path second_root(current_date);
    std::filesystem::path root(dest);
    Paths paths;
    paths.depth = root / second_root / fs::path("depth");
    paths.color = root / second_root / fs::path("color");
    paths.transform = root / second_root / fs::path("transform");
    paths.pointcloud = root / second_root / fs::path("pointcloud");
    fs::create_directories(paths.color);
    fs::create_directories(paths.depth);
    fs::create_directories(paths.transform);
    fs::create_directories(paths.pointcloud);
    return paths;
}

void Integration::save_pointcloud(const PointCloud::Ptr &cloud,
                                  const fs::path &path, int iter) {
    fs::path fn = generatePath(path, iter, ".pcd");
    pcl::io::savePCDFile(fn, *cloud);
}

void Integration::save_img(const std::shared_ptr<Image> &img,
                           const fs::path &path, int iter) {
    fs::path fn = generatePath(path, iter, ".png");
    ROS_DEBUG_STREAM("location: " << fn);
    bool success = o3d::io::WriteImage(fn, *img);
    if (!success) {
        std::stringstream ss;
        ss << "Could not save image at " << fn;
        ROS_WARN_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
}

void Integration::save_transform(const tf::StampedTransform &transform,
                                 const fs::path &path, int iter) {
    fs::path fn = generatePath(path, iter, ".txt");
    std::ofstream file;
    file.open(fn);
    if (!file.is_open()) {
        std::stringstream ss;
        ss << "Could not save transform at " << fn;
        ROS_WARN_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    Eigen::Affine3d tmp;
    tf::transformTFToEigen(transform, tmp);
    file << tmp.matrix() << std::endl;
    file.close();
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
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("/camera_depth_optical_frame", "/base_link",
                                 ros::Time(0), transform);
        transforms.push_back(transform);
    } catch (const tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    pointclouds.push_back(cloud);
}

void Integration::convertPointCloudsToRGBD() {
    std::size_t i(0);
    auto t1 = std::chrono::system_clock::now();
    for (const auto &cloud : pointclouds) {
        auto color = decipherImage(cloud);
        auto depth = decipherDepth(cloud);
        colors.push_back(color);
        depths.push_back(depth);
        if (debug_) {
        save_pointcloud(cloud, paths.pointcloud, i);
        save_img(color, paths.color, i);
        save_img(depth, paths.depth, i);
        save_transform(transforms[i], paths.transform, i);
        }
        ++i;
    }
    auto t2 = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1);
    ROS_WARN_STREAM("The saving of data took: " << diff.count());
}

pcl::PointXYZRGB inline toPointXYZRGB(const Eigen::Vector3d &point,
                                      const Eigen::Vector3d &color) {
    pcl::PointXYZRGB pcl_point;
    pcl_point.x = static_cast<float>(point(0));
    pcl_point.y = static_cast<float>(point(1));
    pcl_point.z = static_cast<float>(point(2));
    pcl_point.r = static_cast<uint8_t>(std::round(color(0) * MAX_UINT));
    pcl_point.g = static_cast<uint8_t>(std::round(color(1) * MAX_UINT));
    pcl_point.b = static_cast<uint8_t>(std::round(color(2) * MAX_UINT));
    return pcl_point;
}

PointCloud::Ptr Integration::toPclPointCloud(
    std::shared_ptr<o3d::geometry::PointCloud> const &cloud) {
    PointCloud::Ptr pcl_cloud(new PointCloud);
    const std::vector<Eigen::Vector3d> &points = cloud->points_;
    const std::vector<Eigen::Vector3d> &colors = cloud->colors_;
    for (std::size_t idx = 0; idx < points.size(); ++idx) {
        auto point = toPointXYZRGB(points[idx], colors[idx]);
        pcl_cloud->push_back(point);
    }
    pcl_cloud->height = 1;
    pcl_cloud->width = points.size();
    return pcl_cloud;
}

void Integration::publishCloud(
    ros::NodeHandle &nh,
    const std::shared_ptr<o3d::geometry::PointCloud> &cloud) {
    auto pcl_cloud = toPclPointCloud(cloud);
    pcl_ros::transformPointCloud(*pcl_cloud, *pcl_cloud,
                                 transforms[0].inverse());
    ros::Publisher pub =
        nh.advertise<PointCloud>("integrate/integratedCloud", 1);
    pcl_cloud->header.frame_id = "base_link";
    ros::Rate loop_rate(4);
    pcl::io::savePCDFileASCII("final_cloud.pcd", *pcl_cloud);
    while (nh.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), pcl_cloud->header.stamp);
        pub.publish(pcl_cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

} // namespace integration
