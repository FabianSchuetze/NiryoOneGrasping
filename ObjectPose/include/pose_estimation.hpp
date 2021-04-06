#ifndef pose_estimation_hpp
#define pose_estimation_hpp
#include <memory>
#include <open3d/Open3D.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <filesystem>
namespace o3d = open3d;
namespace ObjectPose {
typedef o3d::geometry::PointCloud Pointcloud;
//typedef std::shared_ptr<o3d::geometry::PointCloud> PointCloud::Ptr;
using Ptr = std::shared_ptr<o3d::geometry::PointCloud>;
class PoseEstimation {
  public:
    PoseEstimation(const std::filesystem::path&);
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);

  private:
    void cluster();
    void poses();
    std::vector<Ptr> clusters;
    std::vector<Ptr> meshes;
    static std::shared_ptr<o3d::geometry::PointCloud>
    toOpen3DPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
    void findCluster(const Ptr&);
    void readMeshes(const std::filesystem::path&);
};
} // namespace ObjectPose
#endif
