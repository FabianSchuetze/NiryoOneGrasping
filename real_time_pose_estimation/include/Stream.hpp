#ifndef Stream_hpp
#define Stream_hpp
#include <opencv2/core.hpp>
class Stream {
   public:
    Stream() = default;
    virtual bool read(cv::Mat&) = 0;
};
#endif
