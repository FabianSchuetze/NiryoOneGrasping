#ifndef integrate_hpp
#define integrate_hpp
#include <Eigen/Dense>
#include <filesystem>
#include <open3d/Open3D.h>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vector>

namespace o3d = open3d;
namespace integration {
class Integration {
  public:
    struct Paths {
        std::filesystem::path color;
        std::filesystem::path depth;
        std::filesystem::path transform;
    };
    using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
    using RGBDRegistration = std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>;
    explicit Integration(const std::filesystem::path &,
                         const std::filesystem::path &);
    explicit Integration(const std::filesystem::path &);
    void initializePoseGraph();
    std::shared_ptr<o3d::geometry::PointCloud> integrate();
    RGBDRegistration registerImmediateRGBDPair(std::size_t);
    std::shared_ptr<o3d::geometry::PointCloud> createScene();
    void callback(const PointCloud::Ptr &);

  private:
    std::vector<std::shared_ptr<o3d::geometry::Image>> colors;
    std::vector<std::shared_ptr<o3d::geometry::Image>> depths;
    o3d::camera::PinholeCameraIntrinsic intrinsic;
    o3d::pipelines::registration::PoseGraph pose_graph;
    Paths paths;
    std::size_t i;
    static void
    readImages(const std::filesystem::path &,
               std::vector<std::shared_ptr<o3d::geometry::Image>> &);
    void readImages(const std::filesystem::path &, std::string,
                    std::vector<std::shared_ptr<o3d::geometry::Image>> &);
    static o3d::geometry::PointCloud
        meshToPointCloud(o3d::geometry::TriangleMesh);
    static std::shared_ptr<o3d::geometry::RGBDImage>
    convertTORGBD(const o3d::geometry::Image &, const o3d::geometry::Image &,
                  bool);
    void readCameraIntrinsics(const std::filesystem::path &path);
    static std::shared_ptr<o3d::geometry::Image>
    decipherDepth(const PointCloud::Ptr &);
    static std::shared_ptr<o3d::geometry::Image>
    decipherImage(const PointCloud::Ptr &);
    std::string return_current_time_and_date();
    Paths open_folder(const std::string &);
    void save_img(const std::shared_ptr<o3d::geometry::Image> &,
                  const std::filesystem::path &, int);
};
} // namespace integration
#endif
