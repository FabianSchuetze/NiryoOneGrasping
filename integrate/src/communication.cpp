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
#include <utils/utils.hpp>

using o3d::geometry::Image;
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
namespace fs = std::filesystem;
namespace integration {
static constexpr int TOMM = 1000;
static constexpr uint HEIGHT = 480;
static constexpr uint WIDTH = 640;
static constexpr std::size_t MAX_UINT(255);
static constexpr std::size_t TEN(10);
static constexpr std::size_t HUNDRED(100);

std::string return_current_time_and_date() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M");
    return ss.str();
}

fs::path generatePath(const fs::path &root, std::size_t iter,
                      const std::string &ending) {
    fs::path name;
    if (iter < TEN) {
        name = "00" + std::to_string(iter) + ending;
    } else if (iter < HUNDRED) {
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

Integration::Paths Integration::read_folder(const fs::path &pth) {
    ROS_WARN_STREAM("The path is " << pth << std::endl);
    Paths paths;
    paths.depth = pth / fs::path("depth");
    paths.color = pth / fs::path("color");
    paths.transform = pth / fs::path("transform");
    paths.pointcloud = pth / fs::path("pointcloud");
    return paths;
}

void Integration::save_pointcloud(const o3d::geometry::PointCloud &cloud,
                                  const fs::path &pth) {
    o3d::io::WritePointCloud(pth, cloud);
}
void Integration::save_pointcloud(const PointCloud::Ptr &cloud,
                                  const fs::path &path, int iter) {
    fs::path fn = generatePath(path, iter, ".pcd");
    pcl::io::savePCDFile(fn, *cloud);
}

void Integration::save_pose_graph(
    const o3d::pipelines::registration::PoseGraph &pose_graph) const {
    fs::path pose_pth = paths.transform / "pose_graph.json";
    o3d::io::WritePoseGraph(pose_pth, pose_graph);
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

//std::vector<fs::path> Integration::readFiles(const fs::path &root) {
    //auto begin = fs::begin(std::filesystem::directory_iterator(root));
    //auto end = fs::end(std::filesystem::directory_iterator(root));
    //std::vector<fs::path> vec(std::distance(end, begin));
    //if (vec.empty()) {
        //throw std::runtime_error("Path is empty");
    //}
    //std::transform(begin, end, vec.begin(), [](auto x) { return x.path(); });
    //std::sort(vec.begin(), vec.end());
    //return vec;
//}

void Integration::readImages(
    const fs::path &path,
    std::vector<std::shared_ptr<o3d::geometry::Image>> &images) {
    std::vector<fs::path> all_files = utils::filesInFolder(path);
    for (const auto &file : all_files) {
        auto img = std::make_shared<o3d::geometry::Image>();
        o3d::io::ReadImage(file, *img);
        images.push_back(img);
    }
}

void Integration::readTransforms(const fs::path &pth) {
    auto files = utils::filesInFolder(pth);
    const std::string ending(".txt");
    auto filter = [&](const std::string& x) {
        if (ending.size() > x.size())
            return false;
        return std::equal(ending.rbegin(), ending.rend(), x.rbegin());
    }; 
    auto newEnd = std::remove_if(files.begin(), files.end(), filter);
    char sep = ' ';
    for (auto it = files.begin(); it != newEnd; ++it) {
        Eigen::Matrix<double, 4, 4> mat = utils::readMatrix<double, 4, 4>(*it, &sep);
        Eigen::Affine3d tmp;
        tmp = mat;
        //tf::Transform transform{};
        tf::StampedTransform tt{};
        //tt.
        tf::transformEigenToTF(tmp, tt);
        transforms.push_back(tt);
    }
}

void Integration::readFiles() {
    readImages(paths.color, colors);
    readImages(paths.depth, depths);
    readTransforms(paths.transform);
}

void Integration::convertPointCloudsToRGBD() {
    std::size_t i(0);
    auto t1 = std::chrono::system_clock::now();
    if (pointclouds.empty()) {
        throw std::runtime_error("No Pointclouds available");
    }
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

// TODO: Part of utils library
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
    const std::shared_ptr<o3d::geometry::PointCloud> &cloud) {
    auto pcl_cloud = toPclPointCloud(cloud);
    pcl_ros::transformPointCloud(*pcl_cloud, *pcl_cloud,
                                 transforms[0].inverse());
    pcl_cloud->header.frame_id = "base_link";
    ros::Rate loop_rate(4);
    pcl::io::savePCDFileASCII("final_cloud.pcd", *pcl_cloud);
    pcl_conversions::toPCL(ros::Time::now(), pcl_cloud->header.stamp);
    pub.publish(pcl_cloud);
}

} // namespace integration
