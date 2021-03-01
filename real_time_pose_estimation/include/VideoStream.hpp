#ifndef VideoStream_hpp
#define VideoStream_hpp
#include <opencv2/videoio.hpp>
#include "detection_parse_parameters.hpp"
#include "Stream.hpp"
class VideoStream : public Stream {
   public:
    explicit VideoStream(const DetectionParameters&);
    bool read(cv::Mat&) override;

   private:
    cv::VideoCapture cap;  // instantiate VideoCapture
};
#endif
