#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <opencv2/core.hpp>
namespace vision{
void extract_frames(const rs2::frameset&, cv::Mat&, cv::Mat&);
void write_intrinsics_to_file(const rs2::pipeline_profile&, const std::string&);
}
