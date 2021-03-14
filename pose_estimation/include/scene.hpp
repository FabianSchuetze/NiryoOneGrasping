#include <filesystem>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


struct Camera {
    static constexpr float cx = 326.27;
    static constexpr float cy = 237.21;
    static constexpr float fx = 615.4;
    static constexpr float fy = 614.18;
};

class Scene {
    public:
        Scene() = default;
        //explicit Scene(const std::filesystem::path);
        //virtual ~Scene() = default;
        //Scene(const std::filesystem::path&, const std::filesystem::path&); 
        void callback(const PointCloud::Ptr&);
        void decipherImage(const PointCloud::Ptr&);
        void decipherDepth(const PointCloud::Ptr&);
        void estimateFeatures(cv::Ptr<cv::SIFT>&);
        const cv::Mat& descriptors() {return descriptors_;};
    private:
        cv::Mat depth;
        Camera camera;
        cv::Mat img;
        std::vector<cv::KeyPoint> kps;
        cv::Mat descriptors_;
        //cv::Mat points2d;
        cv::Mat points3d;
        std::tuple<float, float, float> deprojectPoint(size_t, size_t) const;
        void create_points();

};
