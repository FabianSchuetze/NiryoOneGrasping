#ifndef model_hpp
#define model_hpp
#include <opencv2/core.hpp>

class Model {
  public:
    Model() = default;
    virtual ~Model() = default;
    virtual const cv::Mat &descriptors() { return descriptors_; };
    virtual const cv::Mat &points3d() { return points3d_; }
    virtual const cv::Mat &img() { return img_; }
    virtual const std::vector<cv::KeyPoint> &kps() { return kps_; }

  protected:
    cv::Mat img_;
    std::vector<cv::KeyPoint> kps_;
    cv::Mat descriptors_;
    cv::Mat points3d_;
};
#endif
