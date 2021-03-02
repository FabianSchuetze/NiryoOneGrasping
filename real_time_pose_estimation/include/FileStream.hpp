#ifndef FileStream_hpp
#define FileStream_hpp
#include <opencv2/core.hpp>
#include <filesystem>
#include "detection_parse_parameters.hpp"
#include "Stream.hpp"
class FileStream : public Stream {
   public:
    explicit FileStream(const DetectionParameters&);
    bool read(cv::Mat&) override;

   private:
    std::filesystem::directory_iterator iter;
    std::filesystem::directory_iterator end;
    void open_folder(const DetectionParameters& paras);
};
#endif
