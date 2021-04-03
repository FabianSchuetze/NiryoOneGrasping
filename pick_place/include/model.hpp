#ifndef model_hpp
#define model_hpp
#include <iostream>
#include <opencv2/core.hpp>

class Model {
  public:
    Model() = default;
    virtual ~Model() = default;
    virtual const cv::Mat &descriptors() const { return matrix(descriptors_); };
    virtual const cv::Mat &points3d() const { return matrix(points3d_); };
    virtual const cv::Mat &img() const { return matrix(img_); }
    virtual const std::vector<cv::KeyPoint> &kps() const { return kps_; }

  protected:
    cv::Mat img_; //NOLINT
    std::vector<cv::KeyPoint> kps_; //NOLINT
    cv::Mat descriptors_; //NOLINT
    cv::Mat points3d_; //NOLINT

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
