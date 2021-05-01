#ifndef utils_hpp
#define utils_hpp
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <tf/LinearMath/Quaternion.h>
namespace utils {
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
std::tuple<double, double, double> RPY(const geometry_msgs::Quaternion &);
} // namespace utils
#endif
