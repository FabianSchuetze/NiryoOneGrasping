#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements,
                      int nInputs, double dt);
void predictKalmanFilter(cv::KalmanFilter &KF, cv::Mat &translation_predicted,
                         cv::Mat &rotation_predicted);
void updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurements,
                        cv::Mat &translation_estimated,
                        cv::Mat &rotation_estimated);
void fillMeasurements(cv::Mat &measurements,
                      const cv::Mat &translation_measured,
                      const cv::Mat &rotation_measured);
