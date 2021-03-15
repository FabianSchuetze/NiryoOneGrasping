#ifndef match_hpp
#define match_hpp
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

class Match {
  public:
    typedef std::vector<cv::DMatch> matches;
    explicit Match(float _ratio) : ratio(_ratio){};
    matches matchDescriptors(const cv::Mat &, const cv::Mat &);
    static std::pair<cv::Mat, cv::Mat>
    corresponding3dPoints(const matches &, const cv::Mat &, const cv::Mat &);
    static void drawMatches(const cv::Mat &, const std::vector<cv::KeyPoint> &,
                            const cv::Mat &, const std::vector<cv::KeyPoint> &,
                            const std::vector<cv::DMatch> &);

  private:
    const float ratio;
    cv::BFMatcher matcher;
    matches RatioTest(std::vector<matches> &&);
};
#endif
