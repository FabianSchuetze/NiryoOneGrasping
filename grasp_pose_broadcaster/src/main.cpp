#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

using param = std::pair<std::string, std::string>;
constexpr std::size_t QUEUE(10);
constexpr double PI(3.14);

template <typename... T>
void readParameters(const ros::NodeHandle &nh, T &... args) {
    auto read_parameters = [&](auto &t) {
        nh.getParam(t.first, t.second);
        if (t.second.empty()) {
            ROS_ERROR_STREAM("Rosparam " << t.first << " not identified");
            throw std::runtime_error("Could not read all parameters");
        }
        ROS_WARN_STREAM("The parameters for " << t.first << " is " << t.second);
    };
    (..., read_parameters(args));
}

double obtain_yaw(const geometry_msgs::Quaternion &quat) {
    double first = 2 * (quat.w * quat.z + quat.x * quat.y);
    double second = 1 - 2 * (quat.x * quat.x + quat.z * quat.z);
    double yaw = std::atan2(first, second);
    if (yaw > PI) {
        yaw = yaw - PI;
    } else if (yaw < -PI) {
        yaw = yaw + PI;
    } // angles are close to zero;
    return yaw;
}

void Callback(const geometry_msgs::PoseArray &msg) {
    tf2_ros::StaticTransformBroadcaster br;
    int i(0);
    std::vector<geometry_msgs::TransformStamped> transforms;
    const auto now = ros::Time::now();
    for (auto const &pose : msg.poses) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = msg.header.frame_id;
        transformStamped.child_frame_id = "object_" + std::to_string(i);
        transformStamped.transform.translation.x = pose.position.x;
        transformStamped.transform.translation.y = pose.position.y;
        transformStamped.transform.translation.z = 0.0;
        double yaw = obtain_yaw(pose.orientation);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        transforms.push_back(transformStamped);
    }
    br.sendTransform(transforms);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_broadcaster");
    ros::NodeHandle nh;
    param incoming_poses("grasp_pose_broadcaster/estimated_poses", "");
    readParameters(nh, incoming_poses);
    ros::Subscriber sub = nh.subscribe(incoming_poses.second, QUEUE, &Callback);
    ros::spin();
    return 0;
};
