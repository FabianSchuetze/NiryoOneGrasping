#include "integrate.hpp"
using namespace o3d::pipelines::registration;
namespace integration {

static constexpr int DEPTH_SCALE(1000);
static constexpr float DEPTH_TRUNC(0.4);
// static constexpr bool CONVERT_RGB_TO_INTENSITY(true);
static constexpr float MAX_DEPTH_DIFF(0.01);
static constexpr float VOXEL_LENGHT(1.0 / 512.0);
static constexpr float SDF_TRUNC(0.4);
Integration::Integration(const std::filesystem::path &path) { ; }

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
    for (std::size_t i = 0; i < pose_graph.nodes_.size(); ++i) {
        auto rgbd = convertTORGBD(colors[i], depths[i], false);
        auto pose = pose_graph.nodes_[i].pose_;
        volume.Integrate(*rgbd, intrinsic, pose.inverse());
    }
    return volume.ExtractPointCloud(); // Does that work
}
Integration::RGBDRegistration
Integration::registerImmediateRGBDPair(std::size_t source) {
    auto source_rgbd = convertTORGBD(colors[source], depths[source], true);
    auto target_rgbd =
        convertTORGBD(colors[source + 1], depths[source + 1], true);
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
    for (std::size_t source = 0; source < colors.size() - 1; ++source) {
        auto [success, trans, info] = registerImmediateRGBDPair(source);
        trans_odometry = trans * trans_odometry;
        Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();
        pose_graph.nodes_.emplace_back(trans_odometry_inv);
        pose_graph.edges_.emplace_back(source, source + 1, trans, info, false);
    }
}

} // namespace integration
