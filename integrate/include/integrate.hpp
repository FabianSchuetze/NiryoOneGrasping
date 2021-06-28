#ifndef integrate_hpp
#define integrate_hpp
#include <Eigen/Dense>
#include <filesystem>
#include <open3d/Open3D.h>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>

namespace o3d = open3d;
namespace integration {
class Integration {
  public:
    struct Paths {
        std::filesystem::path color;
        std::filesystem::path depth;
        std::filesystem::path transform;
        std::filesystem::path pointcloud;
    };
    using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
    using RGBDRegistration = std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>;
    Integration(const std::filesystem::path &, std::string cameraFrame,
                std::string publishTopic, bool debug, ros::NodeHandle &);
    // No ros integration takes rgbd images and writes the integration
    Integration(const std::filesystem::path &,
                const std::filesystem::path &saved_data);
    void initializePoseGraph();
    std::shared_ptr<o3d::geometry::PointCloud> integrate();
    RGBDRegistration registerImmediateRGBDPair(std::size_t);
    std::shared_ptr<o3d::geometry::PointCloud> createScene();
    void callback(const PointCloud::Ptr &);
    void publishCloud(const std::shared_ptr<o3d::geometry::PointCloud> &);
    void convertPointCloudsToRGBD();
    void readFiles();
    Paths paths;
    static void save_pointcloud(const o3d::geometry::PointCloud &,
                                const std::filesystem::path &);

  private:
    std::vector<std::shared_ptr<o3d::geometry::Image>> colors;
    std::vector<std::shared_ptr<o3d::geometry::Image>> depths;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointclouds;
    std::vector<tf::StampedTransform> transforms;
    o3d::camera::PinholeCameraIntrinsic intrinsic;
    o3d::pipelines::registration::PoseGraph pose_graph;
    ros::Publisher pub;
    bool debug_;
    const std::string cameraFrame, publishTopic;
    tf::TransformListener listener;
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
    Paths open_folder(const std::string &);
    Paths read_folder(const std::filesystem::path &);
    static void save_img(const std::shared_ptr<o3d::geometry::Image> &,
                         const std::filesystem::path &, int);
    void save_pose_graph(const o3d::pipelines::registration::PoseGraph &) const;
    static void save_pointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &,
                                const std::filesystem::path &, int);
    static void save_transform(const tf::StampedTransform &,
                               const std::filesystem::path &, int);
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    toPclPointCloud(std::shared_ptr<o3d::geometry::PointCloud> const &cloud);
    void readImages(const std::filesystem::path &,
                    std::vector<std::shared_ptr<o3d::geometry::Image>> &);
    void readTransforms(const std::filesystem::path&);
    
};
} // namespace integration
#endif
