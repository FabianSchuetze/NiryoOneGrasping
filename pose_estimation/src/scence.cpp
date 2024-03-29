#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <opencv2/features2d.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf_conversions/tf_eigen.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "scene.hpp"

using namespace PoseEstimation;

// constexpr int TO_DEPTH = 100;
static constexpr int TOMM = 1000;
static constexpr uint HEIGHT = 480;
static constexpr uint WIDTH = 640;
static constexpr uint MIN_VALUES = 100;
static constexpr uint MAX_ATTMEPTS = 100;
static constexpr double SLEEP(0.1);

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
    //TODO: Add debug variable specifying whether to show image
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

template <typename T> void writeToFile(const T &mat, const std::string &fn) {
    std::ofstream file(fn);
    if (file.is_open()) {
        file << mat;
    }
}

void Scene::estimateFeatures(const cv::Ptr<cv::SIFT> &estimator) {
    kps_.clear();
    estimator->detectAndCompute(img_, cv::noArray(), kps_, descriptors_);
    create_points();
    Eigen::Affine3d points3d, points3d_out;
    std::size_t rows = points3d_.rows;
    Eigen::MatrixXd eigen_mat(Eigen::MatrixXd::Zero(rows, 3));
    cv::cv2eigen(points3d_, eigen_mat);
    geometry_msgs::TransformStamped transform;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::size_t i(0);
    while (i < MAX_ATTMEPTS) {
        try {
            transform = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame",
                                                 ros::Time(0));
            break;
        } catch (const tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(SLEEP).sleep();
            ++i;
        }
    }
    if (i == MAX_ATTMEPTS) {
        throw std::runtime_error("Could not find transfrom");
    }
    Eigen::Isometry3d trans(Eigen::Matrix4d::Identity(4, 4));
    Eigen::Affine3d tmp_ = tf2::transformToEigen(transform.transform);
    trans.matrix() = tmp_.matrix();
    Eigen::MatrixXd out = trans * eigen_mat.transpose().colwise().homogeneous();
    Eigen::MatrixXd tmp = out.transpose();
    //writeToFile(eigen_mat, "incoming_points.txt");
    //writeToFile(trans.matrix(), "transform.txt");
    //writeToFile(tmp, "out.txt");
    cv::eigen2cv(tmp, points3d_);
}

