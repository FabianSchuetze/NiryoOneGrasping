#include "integrate.hpp"
#include <ros/ros.h>
#include <thread>
using namespace o3d::pipelines::registration;
namespace fs = std::filesystem;
namespace integration {

static constexpr int DEPTH_SCALE(1000);
static constexpr float DEPTH_TRUNC(0.4);
static constexpr float MAX_DEPTH_DIFF(0.01);
static constexpr float VOXEL_LENGHT(1.0 / 512.0);
static constexpr float SDF_TRUNC(0.04);
static constexpr int LOG_FREQUENCY(10);

void Integration::readCameraIntrinsics(const fs::path &path) {
    if (fs::exists(path)) {
        if (!o3d::io::ReadIJsonConvertible(path, intrinsic)) {
            std::stringstream ss;
            ss << "Cannot Read file: " << path << " for intrinsics";
            ROS_WARN_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    } else {
        std::stringstream ss;
        ss << "Cannot open path: " << path;
        ROS_WARN_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
}

Integration::Integration(const fs::path &camera,
                         std::string _cameraFrame,
                         // const std::string &_cameraFrame,
                         std::string _publishTopic, bool debug,
                         ros::NodeHandle &nh)
    : cameraFrame(std::move(_cameraFrame)),
      publishTopic(std::move(_publishTopic)), debug_(debug) {
    readCameraIntrinsics(camera);
    paths = open_folder(
        "/home/fabian/Documents/work/transforms/src/integrate/data");
    pub = nh.advertise<PointCloud>("integrate/integratedCloud", 1, true);
}

std::shared_ptr<o3d::geometry::RGBDImage>
Integration::convertTORGBD(const o3d::geometry::Image &color,
                           const o3d::geometry::Image &depth,
                           bool convert_rgb_to_intensity) {
    auto rgbd = o3d::geometry::RGBDImage::CreateFromColorAndDepth(
        color, depth, DEPTH_SCALE, DEPTH_TRUNC, convert_rgb_to_intensity);
    return rgbd;
}

std::shared_ptr<o3d::geometry::PointCloud> Integration::integrate() {
    auto volume = o3d::pipelines::integration::ScalableTSDFVolume(
        VOXEL_LENGHT, SDF_TRUNC,
        o3d::pipelines::integration::TSDFVolumeColorType::RGB8);
    const std::size_t sz = pose_graph.nodes_.size();
    for (std::size_t i = 0; i < sz; ++i) {
        if ((i % LOG_FREQUENCY) == 0) {
            ROS_WARN_STREAM("Integrating image " << i << "/" << sz);
        }
        auto rgbd = convertTORGBD(*colors[i], *depths[i], false);
        auto pose = pose_graph.nodes_[i].pose_;
        volume.Integrate(*rgbd, intrinsic, pose.inverse());
    }
    return volume.ExtractPointCloud();
}

Integration::RGBDRegistration
Integration::registerImmediateRGBDPair(std::size_t source) {
    auto source_rgbd = convertTORGBD(*colors[source], *depths[source], true);
    auto target_rgbd =
        convertTORGBD(*colors[source + 1], *depths[source + 1], true);
    auto option = o3d::pipelines::odometry::OdometryOption();
    option.max_depth_diff_ = MAX_DEPTH_DIFF;
    auto [success, trans, info] = o3d::pipelines::odometry::ComputeRGBDOdometry(
        *source_rgbd, *target_rgbd, intrinsic, Eigen::Matrix4d::Identity(4, 4),
        o3d::pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(), option);
    return {success, trans, info};
}

void Integration::initializePoseGraph() {
    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity(4, 4);
    pose_graph.nodes_.emplace_back(trans_odometry);
    std::size_t sz = colors.size();
    fs::path pose_pth("/home/fabian/Documents/work/transforms/src/integrate/"
                      "data/pose_graph.json");
    for (std::size_t source = 0; source < sz - 1; ++source) {
        if ((source % LOG_FREQUENCY) == 0) {
            ROS_WARN_STREAM("Registering image " << source << "/" << sz);
        }
        auto [success, trans, info] = registerImmediateRGBDPair(source);
        if (!success) {
            ROS_WARN_STREAM("Could not align frames" << source << "with "
                                                     << source + 1);
        }
        trans_odometry = trans * trans_odometry;
        Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();
        pose_graph.nodes_.emplace_back(trans_odometry_inv);
        pose_graph.edges_.emplace_back(source, source + 1, trans, info, false);
    }
    o3d::io::WritePoseGraph(pose_pth, pose_graph);
}

std::shared_ptr<o3d::geometry::PointCloud> Integration::createScene() {
    auto t1 = std::chrono::system_clock::now();
    initializePoseGraph();
    auto pcd = integrate();
    auto t2 = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1);
    ROS_WARN_STREAM("The interation took: " << diff.count());
    return pcd;
}
} // namespace integration
