#ifndef pickplace_scene_hpp
#define pickplace_scene_hpp
#include "model.hpp"
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <string>
namespace fs = std::filesystem;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

struct Camera {
    static constexpr float cx = 326.27;
    static constexpr float cy = 237.21;
    static constexpr float fx = 615.4;
    static constexpr float fy = 614.18;
};

class Scene : public Model {
  public:
    void callback(const PointCloud::Ptr &);
    void decipherImage(const PointCloud::Ptr &);
    void decipherDepth(const PointCloud::Ptr &);
    const cv::Mat &depth() { return depth_; }

  private:
    cv::Mat depth_;
    Camera camera;
};
#endif

