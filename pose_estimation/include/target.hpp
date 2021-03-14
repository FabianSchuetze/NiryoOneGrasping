#include <vector>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

class Target {
    public:
        Target(const std::filesystem::path&, const std::filesystem::path&);
        const cv::Mat& descriptors() {return descriptors_;};
    private:
        cv::Mat descriptors_;
        cv::Mat points3d;
        //cv::Mat points2d;
        std::vector<cv::KeyPoint> kps;
        cv::Mat img;
};
