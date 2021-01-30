#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <opencv2/core.hpp>
#include <string>
namespace vision {
void extract_frames(const rs2::frameset&, cv::Mat&, cv::Mat&);
void write_intrinsics_to_file(const rs2::pipeline_profile&, const std::string&);
void create_folders_if_neccessary(const std::string&);
void write_frames_to_file(cv::Mat, cv::Mat, const std::string&, size_t);
struct Paras {
    std::string root_dir;
    std::string transform_file;
};
Paras get_paras(const ros::NodeHandle&);
const Eigen::Affine3d read_hand_to_eye_transform(const std::string&);
bool obtain_transform(std::string, std::string, const tf2_ros::Buffer&,
                      Eigen::Affine3d&);
//void base_to_camera_transform(const eigen::matrix4d&,
                              //const geometry_msgs::transformstamped&,
                              //eigen::matrix4d&);
}  // namespace vision
