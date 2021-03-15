#include "model.hpp"
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

struct Camera {
    static constexpr float cx = 326.27;
    static constexpr float cy = 237.21;
    static constexpr float fx = 615.4;
    static constexpr float fy = 614.18;
};

class Scene : public Model {
  public:
    Scene() = default;
    ~Scene() override = default;
    void deserialize(const std::filesystem::path &,
                     const std::filesystem::path &);
    void callback(const PointCloud::Ptr &);
    void decipherImage(const PointCloud::Ptr &);
    void decipherDepth(const PointCloud::Ptr &);
    void estimateFeatures(cv::Ptr<cv::SIFT> &);

  private:
    cv::Mat depth_;
    Camera camera;
    std::tuple<float, float, float> deprojectPoint(size_t, size_t) const;
    void create_points();
};
