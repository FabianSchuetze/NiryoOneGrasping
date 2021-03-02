#ifndef RSStream_hpp
#define RSStream_hpp
#include <filesystem>
#include <librealsense2/rs.hpp>

#include "Stream.hpp"
#include "detection_parse_parameters.hpp"
class RSStream : public Stream {
   public:
    explicit RSStream(const DetectionParameters&);
    bool read(cv::Mat&) override;

   private:
    void extract_frames(const rs2::frameset&, cv::Mat&);
    rs2::pipeline pipe;
    rs2::align align_to;
    std::filesystem::path folder;
    int iter;
    void open_folder(const DetectionParameters&);
};
#endif
