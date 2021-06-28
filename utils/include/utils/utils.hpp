#ifndef utils_hpp
#define utils_hpp
#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <tf/LinearMath/Quaternion.h>
#include <filesystem>
#include <fstream>

namespace utils {

template <typename T>
inline bool convert(const std::string&, T&);

template <>
inline bool convert(const std::string& tmp, double& out) {
    try {
        out = std::stod(tmp);
        return true;
    } catch (...) {
        return false;
    }
}

template <>
inline bool convert(const std::string& tmp, float& out) {
    try {
        out = std::stof(tmp);
        return true;
    } catch (...) {
        return false;
    }
}
template <typename... T>
void readParameters(const ros::NodeHandle &nh, T &... args) {
    auto read_parameters = [&](auto &t) {
        nh.getParam(t.first, t.second);
        if constexpr (std::is_same_v<decltype(t), std::string>) {
            if (t.second.empty()) {
                ROS_WARN_STREAM("Rosparam " << t.first << " not identified");
                throw std::runtime_error("Could not read all parameters");
            }
        }
        ROS_WARN_STREAM("The parameters for " << t.first << " is " << t.second);
    };
    (..., read_parameters(args));
}
class DOF{
    public:
        //explicit DOF(const geometry_msgs::Pose& pose);
        DOF(double, double, double, double, double, double);
        geometry_msgs::TransformStamped transformStamped() const;
        geometry_msgs::Pose pose() const;
        double x, y, z, roll, pitch, yaw;
};
//geometry_msgs::TransformStamped generateTransformation(const DOF &);
std::tuple<double, double, double> RPY(const tf::Quaternion & quat);
std::tuple<double, double, double> RPY(const Eigen::Isometry3d &);
std::tuple<double, double, double> RPY(const Eigen::Matrix4d &);
std::tuple<double, double, double> RPY(const geometry_msgs::Quaternion &);
std::string shortName(const std::string &, const std::string &);

std::vector<std::filesystem::path> filesInFolder(const std::filesystem::path &root);


template <typename T, int rows, int cols>
Eigen::Matrix<T, rows, cols> readMatrix(const std::filesystem::path& pth,
                                        const char* sep) {
    using MatrixType = Eigen::Matrix<T, rows, cols>;
    std::ifstream infile;
    infile.open(pth);
    std::array<T, rows * cols> buffer{};
    std::size_t row(0);
    std::string line;
    while ((std::getline(infile, line)) and (row < rows)) {
        std::istringstream ss(line);
        std::string token;
        std::size_t col(0);
        while ((std::getline(ss, token, *sep)) and (col < cols)) {
            T res{};
            if (convert<T>(token, res)) {
                buffer.at(row * rows + col) = res;
                ++col;
            }
        }
        ++row;
    }
    infile.close();
    MatrixType mat{};
    mat = Eigen::Map<Eigen::Matrix<T, rows, cols, Eigen::RowMajor>>(
        buffer.data(), rows, cols);
    return mat;
}

} // namespace utils
#endif
