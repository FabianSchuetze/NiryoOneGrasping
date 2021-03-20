#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "scene.hpp"


// constexpr int TO_DEPTH = 100;
constexpr int TOMM = 1000;
constexpr uint HEIGHT = 480;
constexpr uint WIDTH = 640;
constexpr uint MIN_VALUES = 100;

void Scene::decipherImage(const PointCloud::Ptr &cloud) {
    sensor_msgs::Image ros_img;
    pcl::toROSMsg(*cloud, ros_img);
    cv_bridge::CvImagePtr tmp_img =
        cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    if (tmp_img->image.empty()) {
        std::cout << "Could not read the image: " << std::endl;
    }
    img_ = tmp_img->image;
}

void Scene::decipherDepth(const PointCloud::Ptr &cloud) {
    if (cloud->height != HEIGHT) {
        throw std::runtime_error("Has different height");
    }
    if (cloud->width != WIDTH) {
        throw std::runtime_error("Has different width");
    }
    depth_ = cv::Mat(HEIGHT, WIDTH, CV_16UC1);
    for (size_t row = 0; row < HEIGHT; ++row) {
        for (size_t column = 0; column < WIDTH; ++column) {
            const auto &pt = cloud->at(column, row);
            auto val = static_cast<ushort>(std::round(pt.z * TOMM)); // NOLINT
            depth_.at<ushort>(row, column) = val;
        }
    }
}

void Scene::callback(const PointCloud::Ptr &point_cloud) {
    decipherImage(point_cloud);
    decipherDepth(point_cloud);
    if (!img().empty()) {
        cv::imshow("test", img());
        cv::waitKey(1);
    }
}

void Scene::deserialize(const fs::path &color_pth, const fs::path &depth_pth) {
    img_ = cv::imread(color_pth);
    depth_ = cv::imread(depth_pth, cv::IMREAD_ANYDEPTH);
}

std::tuple<float, float, float, int16_t> inline Scene::deprojectPoint(
    size_t x, size_t y) const {
    int16_t val = depth_.at<ushort>(y, x);
    float p_z = static_cast<float>(depth_.at<ushort>(y, x)) / TOMM;
    float p_x = (static_cast<float>(x) - Camera::cx) * p_z / Camera::fx;
    float p_y = (static_cast<float>(y) - Camera::cy) * p_z / Camera::fy;
    return {p_x, p_y, p_z, val};
}

void Scene::create_points() {
    points3d_ = cv::Mat(kps_.size(), 3, CV_32FC1);
    size_t i(0), invalid_depth_counter(0);
    for (const cv::KeyPoint &kp : kps_) {
        uint x = static_cast<uint>(std::round(kp.pt.x));
        uint y = static_cast<uint>(std::round(kp.pt.y));
        const auto [p_x, p_y, p_z, depth] = deprojectPoint(x, y);
        points3d_.at<float>(i, 0) = p_x;
        points3d_.at<float>(i, 1) = p_y;
        points3d_.at<float>(i, 2) = p_z;
        if (depth < 1) {
            invalid_depth_counter++;
        }
        ++i;
    }
    if (invalid_depth_counter > MIN_VALUES) {
        ROS_WARN_STREAM("Many depth values are zero. Camera working correctly");
    }
}

void Scene::estimateFeatures(const cv::Ptr<cv::SIFT> &estimator) {
    kps_.clear();
    estimator->detectAndCompute(img_, cv::noArray(), kps_, descriptors_);
    create_points();
}

