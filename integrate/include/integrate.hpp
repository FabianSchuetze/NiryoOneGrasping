#ifndef integrate_hpp
#define integrate_hpp
#include <Eigen/Dense>
#include <filesystem>
#include <open3d/Open3D.h>
#include <vector>
namespace o3d = open3d;
namespace integration {
class Integration {
  public:
    typedef std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> RGBDRegistration;
    explicit Integration(const std::filesystem::path &);
    void initializePoseGraph();
    o3d::geometry::PointCloud integrate();
    RGBDRegistration registerImmediateRGBDPair(std::size_t, std::size_t);

  private:
    std::vector<o3d::geometry::Image> color;
    std::vector<o3d::geometry::Image> depth;
    o3d::camera::PinholeCameraIntrinsic intrinic;
    o3d::pipelines::registration::PoseGraph pose_graph;
    void readImages(const std::filesystem::path &);
    static o3d::geometry::PointCloud
        meshToPointCloud(o3d::geometry::TriangleMesh);
};
} // namespace integration
#endif
