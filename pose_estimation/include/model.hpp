#ifndef model_hpp
#define model_hpp
#include <iostream>
#include <opencv2/core.hpp>

class Model {
  public:
    Model() = default;
    virtual ~Model() = default;
    virtual const cv::Mat &descriptors() { return matrix(descriptors_); };
    virtual const cv::Mat &points3d() { return matrix(points3d_); };
    virtual const cv::Mat &img() { return matrix(img_); }
    virtual const std::vector<cv::KeyPoint> &kps() { return kps_; }

  protected:
    cv::Mat img_;
    std::vector<cv::KeyPoint> kps_;
    cv::Mat descriptors_;
    cv::Mat points3d_;

  private:
    static const cv::Mat &matrix(const cv::Mat &mat) {
        if ((mat.rows == 0) || (mat.cols == 0)) {
            std::cerr
                << "Warning: The matrix to be accessed has zero elemtents\n";
        }
        return mat;
    }
};
#endif
