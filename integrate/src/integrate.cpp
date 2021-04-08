#include "integrate.hpp"
using namespace o3d::pipelines::registration;
namespace integration {
Integration::Integration(const std::filesystem::path &path) { ; }
void Integration::initializePoseGraph() {
    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity(4, 4);
    pose_graph.nodes_.emplace_back(trans_odometry);
    for (std::size_t source = 0; source < color.size() - 1; ++source) {
        const auto [success, trans, info] =
            registerImmediateRGBDPair(source, source + 1);
        trans_odometry = trans * trans_odometry;
        Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();
        pose_graph.nodes_.emplace_back(trans_odometry_inv);
        pose_graph.edges_.emplace_back(source, source + 1, trans, info, false);
    }
}

} // namespace integration
