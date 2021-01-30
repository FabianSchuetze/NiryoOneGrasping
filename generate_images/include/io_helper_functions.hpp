#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
namespace vision{
void extract_frames(const rs2::frameset&, cv::Mat&, cv::Mat&);
void write_intrinsics_to_file(const rs2::pipeline_profile&, const std::string&);
void create_folders_if_neccessary(const std::string&);
void write_frames_to_file(cv::Mat, cv::Mat, const std::string&, size_t);
struct Paras {
    std::string root_dir;
    std::string transform_file;
};
Paras get_paras(const ros::NodeHandle&);
const Eigen::Matrix4d read_hand_to_eye_transform(const std::string&);
}
