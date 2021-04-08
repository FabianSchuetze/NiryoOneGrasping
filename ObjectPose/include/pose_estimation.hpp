#ifndef pose_estimation_hpp
#define pose_estimation_hpp
#include <filesystem>
#include <memory>
#include <open3d/Open3D.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
namespace o3d = open3d;
namespace ObjectPose {
typedef o3d::geometry::PointCloud Pointcloud;
using Ptr = std::shared_ptr<o3d::geometry::PointCloud>;
class PoseEstimation {
  public:
    explicit PoseEstimation(const std::filesystem::path &);
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);

  private:
    struct BestResult {
        int source_idx = -1;
        int target_idx = -1;
        o3d::pipelines::registration::RegistrationResult result;
        Ptr source;
        Ptr target;
    };
    void cluster();
    void poses();
    std::vector<Ptr> clusters;
    std::vector<o3d::geometry::TriangleMesh> meshes;
    static std::shared_ptr<o3d::geometry::PointCloud>
    toOpen3DPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
    void findCluster(const Ptr &);
    void readMeshes(const std::filesystem::path &);
    static o3d::pipelines::registration::RegistrationResult
    globalRegistration(const Ptr &, const Ptr &);
    BestResult
    estimateTransformations(std::vector<o3d::geometry::TriangleMesh> &,
                            std::vector<Ptr> &);
    std::vector<BestResult> estimateTransformations();
};
} // namespace ObjectPose
#endif
