#ifndef pose_estimation_hpp
#define pose_estimation_hpp
#include <filesystem>
#include <memory>
#include <open3d/Open3D.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
namespace o3d = open3d;
namespace ObjectPose {
typedef o3d::geometry::PointCloud Pointcloud;
using Ptr = std::shared_ptr<o3d::geometry::PointCloud>;
double calculateYaw(const Eigen::Matrix4d&);
double calculateRoll(const Eigen::Matrix4d&);
double calculatePitch(const Eigen::Matrix4d&);
class PoseEstimation {
  public:
    explicit PoseEstimation(const std::filesystem::path &, const std::string &,
                            ros::NodeHandle &);
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);

  private:
    struct BestResult {
        int source_idx = -1;
        int target_idx = -1;
        std::string source_name;
        o3d::pipelines::registration::RegistrationResult result;
        Ptr source;
        Ptr target;
    };
    std::vector<Ptr> clusters;
    std::vector<std::pair<std::string, o3d::geometry::TriangleMesh>> meshes;
    ros::Publisher publisher;
    std::size_t callback_received;
    void cluster();
    void poses();
    static std::shared_ptr<o3d::geometry::PointCloud>
    toOpen3DPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
    static std::vector<Ptr> findCluster(const Ptr &);
    void readMeshes(const std::filesystem::path &);
    o3d::pipelines::registration::RegistrationResult
    globalRegistration(const Ptr &, const Ptr &);
    BestResult estimateTransformations(
        std::vector<std::pair<std::string, o3d::geometry::TriangleMesh>> &,
        std::vector<Ptr> &);
    std::vector<BestResult> estimateTransformations();
    void publishTransforms(const std::vector<BestResult> &);
    o3d::pipelines::registration::RegistrationResult
    ModifiedRegistrationRANSACBasedOnFeatureMatching(
        const o3d::geometry::PointCloud &source,
        const o3d::geometry::PointCloud &target,
        const o3d::pipelines::registration::Feature &source_feature,
        const o3d::pipelines::registration::Feature &target_feature,
        bool mutual_filter, double max_correspondence_distance,
        const o3d::pipelines::registration::TransformationEstimation &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        int ransac_n /* = 3*/,
        const std::vector<std::reference_wrapper<
            const o3d::pipelines::registration::CorrespondenceChecker>>
            &checkers /* = {}*/,
        const o3d::pipelines::registration::RANSACConvergenceCriteria &criteria
        /* = RANSACConvergenceCriteria()*/);
};
} // namespace ObjectPose
#endif
