#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "scene.hpp"

constexpr int TOCM = 100;
constexpr uint HEIGHT = 480;
constexpr uint WIDTH = 640;
// Scene read_scene_static() {
// Scene scene;
// return scene;

//// Scene(
//}

// Scene read_scene_dynamic() {
// Scene scene;
// ros::NodeHandle n;
// ros::ServiceClient client =
// n.serviceClient<PointCloud::Ptr>(read_scene_dynamic Scene scene; return
// scene;
//}
//

void Scene::decipher_image(const PointCloud::Ptr& cloud) {
    sensor_msgs::Image ros_img;
    pcl::toROSMsg(*cloud, ros_img);
    cv_bridge::CvImagePtr tmp_img =
        cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    if (tmp_img->image.empty()) {
        std::cout << "Could not read the image: " << std::endl;
    }
    img = tmp_img->image;
}

void Scene::decipher_depth(const PointCloud::Ptr& cloud) {
    if (cloud->height != HEIGHT) {
        throw std::runtime_error("Has different height");
    }
    if (cloud->width != WIDTH) {
        throw std::runtime_error("Has different width");
    }
    depth = cv::Mat(HEIGHT, WIDTH, CV_16UC1);
    for (size_t row = 0; row < HEIGHT; ++row) {
        for (size_t column = 0; column < WIDTH; ++column) {
            const auto& pt = cloud->at(column, row);
            depth.at<ushort>(row, column) = pt.z / TOCM;
        }
    }
}

void Scene::callback(const PointCloud::Ptr& point_cloud) {
    decipher_image(point_cloud);
    decipher_depth(point_cloud);
}
