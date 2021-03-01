#ifndef RSStream_hpp
#define RSStream_hpp
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API

#include "detection_parse_parameters.hpp"
#include "Stream.hpp"
class RSStream : public Stream {
   public:
    explicit RSStream(const DetectionParameters&);
    bool read(cv::Mat&) override;

   private:
    void extract_frames(const rs2::frameset&, cv::Mat&);
    rs2::pipeline pipe;
    rs2::align align_to;
};
#endif
