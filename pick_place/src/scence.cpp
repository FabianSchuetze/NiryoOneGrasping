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
