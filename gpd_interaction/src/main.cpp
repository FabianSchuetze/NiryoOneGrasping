#include <chrono>
#include <filesystem>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <gpd_ros/GraspConfigList.h>
#include <object_pose/positions.h>
#include <open3d/Open3D.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>

using param = std::pair<std::string, std::string>;
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
namespace fs = std::filesystem;
constexpr std::size_t QUEUE(10);
constexpr double PI(3.14);
constexpr double HEIGHT_BAKING(0.08);
static constexpr std::size_t MAX_UINT(255);

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


double calculateRoll(const Eigen::Quaterniond &quat) {

    //Eigen::Matrix3d tmp = transformation.matrix().block(0, 0, 3, 3);
    //Eigen::Quaternion<double> quat(tmp);
    double first = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
    double second = 1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z());
    double roll = std::atan2(first, second);
    return roll;
}
double calculateYaw(const Eigen::Quaterniond &quat) {

    //Eigen::Matrix3d tmp = transformation.matrix().block(0, 0, 3, 3);
    //Eigen::Quaternion<double> quat(tmp);
    double first = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
    double second = 1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z());
    double yaw = std::atan2(first, second);
    return yaw;
}

double calculateYaw(const geometry_msgs::Quaternion &quat) {
    double first = 2 * (quat.w * quat.z + quat.x * quat.y);
    double second = 1 - 2 * (quat.x * quat.x + quat.z * quat.z);
    double yaw = std::atan2(first, second);
    return yaw;
}

double calculatePitch(const Eigen::Quaterniond &quat) {

    double first = 2 * (quat.w() * quat.y() - quat.y() * quat.x());
    double pitch = std::asin(first);
    return pitch;
}

std::tuple<double, double, double> RPY(const Eigen::Isometry3d &transform) {
    Eigen::Matrix3d tmp = transform.matrix().block(0, 0, 3, 3);
    Eigen::Quaternion<double> quat(tmp);
    double roll = calculateRoll(quat);
    double pitch = calculatePitch(quat);
    double yaw = calculateYaw(quat);
    return {roll, pitch, yaw};
}

Eigen::Isometry3d generateTransformation(const geometry_msgs::Pose &pose,
                                         double yaw) {
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose grasp_pose;
    transformStamped.transform.translation.x = pose.position.x;
    transformStamped.transform.translation.y = pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    Eigen::Isometry3d transform = tf2::transformToEigen(transformStamped);
    return transform;
}

class SubscribeAndPublish {
  public:
    SubscribeAndPublish(ros::NodeHandle &n_, const std::string &publication) {
        // ROS_WARN_STREAM("The publication is" << publication);
        pub_ = n_.advertise<geometry_msgs::PoseArray>(publication, 1, true);
        publish_pointcloud_ =
            n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/cloud_pcd", 1);
    }
    Eigen::Isometry3d
    convertHandToTransform(const gpd_ros::GraspConfig &grasp) {
        Eigen::Isometry3d trans;
        Eigen::Matrix4d frame = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Vector3d approach, binormal, axis, position;
        tf2::fromMsg(grasp.approach, approach);
        tf2::fromMsg(grasp.binormal, binormal);
        tf2::fromMsg(grasp.axis, axis);
        tf2::fromMsg(grasp.position, position);
        frame.block<3, 1>(0, 0) = approach;
        frame.block<3, 1>(0, 1) = binormal;
        frame.block<3, 1>(0, 2) = axis;
        frame.block<3, 1>(0, 3) = position;
        trans.matrix() = frame;
        return trans;
    }

    void callback_grasp_pose(const gpd_ros::GraspConfigList &msg) {
        possible_transforms.clear();
        for (auto const &grasp : msg.grasps) {
            auto possible_transform = convertHandToTransform(grasp);
            possible_transforms.push_back(possible_transform);
        }
        grasp_pose_received = true;
    }

    pcl::PointXYZRGB inline toPointXYZRGB(const Eigen::Vector3d &point) {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = static_cast<float>(point(0));
        pcl_point.y = static_cast<float>(point(1));
        pcl_point.z = static_cast<float>(point(2));
        pcl_point.r = static_cast<uint8_t>(0);
        pcl_point.g = static_cast<uint8_t>(0);
        pcl_point.b = static_cast<uint8_t>(255);
        return pcl_point;
    }

    std::string shortName(const std::string &input_name,
                          std::string extension) {
        const std::string fn = fs::path(input_name).filename();
        std::string delimiter = "_";
        std::string token = fn.substr(0, fn.find(delimiter));
        return token + extension;
    }

    void publishPointCloud(const std::string &name) {
        open3d::geometry::TriangleMesh mesh;
        bool success = open3d::io::ReadTriangleMesh(name, mesh);
        if (!success) {
            ROS_ERROR_STREAM("Could not read mesh file " << name);
            throw std::runtime_error("Couldd not read mesh file");
        }
        auto cloud = mesh.SamplePointsUniformly(50000);
        PointCloud::Ptr pcl_cloud(new PointCloud);
        for (const auto &o3d_point : cloud->points_) {
            auto point = toPointXYZRGB(o3d_point);
            pcl_cloud->push_back(point);
        }
        pcl_cloud->height = 1;
        pcl_cloud->width = cloud->points_.size();
        std::string fn = fs::path(name).filename();
        pcl_cloud->header.frame_id = fn;
        publish_pointcloud_.publish(pcl_cloud);
    }

    void
    publishTransform(const Eigen::Isometry3d &transform,
                     const std::string &name, std::string extension,
                     std::vector<geometry_msgs::TransformStamped> &transforms) {
        auto tmp = tf2::eigenToTransform(transform);
        tmp.header.frame_id = "base_link";
        tmp.child_frame_id = shortName(name, extension);
        transforms.push_back(tmp);
    }

    bool feasible(const Eigen::Isometry3d &frame) {
        Eigen::MatrixXd gripper_frame = Eigen::MatrixXd::Identity(4, 4);
        gripper_frame(0, 3) = 0.12;
        Eigen::Matrix4d result;
        result = frame.matrix() * gripper_frame;
        bool is_feasible = (result(2, 3) > 0.02);
        return is_feasible;
    }

    Eigen::Isometry3d
    filterPossibleTransforms(const Eigen::Isometry3d &object) {
        Eigen::Isometry3d best_hand;
        double error = 1000;
        for (const auto &possible_transform : possible_transforms) {
            const auto hand = object * possible_transform;
            if (!feasible(hand)) {
                if (hand.matrix()(2,3) > 0.01) {
                    return hand;
                }
                continue;
            }
            std::cout << "feasible" << std::endl;
            auto [roll_object, pitch_object, yaw_object] = RPY(object);
            auto [roll_hand, pitch_hand, yaw_hand] = RPY(hand);
            double diff_yaw = std::abs(yaw_object - yaw_hand);
            double diff_roll = std::abs(roll_object - roll_hand);
            double curr_error = diff_yaw + diff_roll;
            if (curr_error < error) {
                std::cout << "yaw_object and hand" << yaw_hand  << ", " <<
                    yaw_object << std::endl;
                best_hand = possible_transform;
                error = curr_error;
            }
        }
        return best_hand;
    }

    void callback_object_pose(const object_pose::positions &msg) {
        grasp_pose_received = false;
        ROS_WARN_STREAM("Inside the object pose callback");
        // const auto now = ros::Time::now();
        std::vector<geometry_msgs::TransformStamped> transforms;
        // geometry_msgs::PoseArray poses;
        // poses.header.frame_id = msg.poses.header.frame_id;
        // poses.header.stamp = now;
        for (std::size_t i = 0; i < msg.objects.size(); ++i) {
            grasp_pose_received = false;
            geometry_msgs::Pose pose = msg.poses.poses[i];
            const std::string &name = msg.objects[i];
            double yaw = calculateYaw(pose.orientation);
            Eigen::Isometry3d object_frame = generateTransformation(pose, yaw);
            publishTransform(object_frame, name, "", transforms);
            publishPointCloud(name);
            while ((!grasp_pose_received) and ros::ok()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            grasp_pose_received = false;
            int j = 0;
            for (const auto& hand : possible_transforms) {
            //Eigen::Isometry3d hand = filterPossibleTransforms(object_frame);
                Eigen::Isometry3d tmp = object_frame * hand;
                publishTransform(tmp, name, "_grasp" + std::to_string(j), transforms);
                j++;
                if (j > 9) {
                    break;
                }
            }
            // auto final_transform = tf2::eigenToTransform(tmp);
            // final_transform.child_frame_id = shortName(name, "_grasp");
            // final_transform.header.frame_id = "base_link";
            // transforms.push_back(final_transform);
        }
        ROS_WARN_STREAM("Transfroms size: " << transforms.size());
        // pub_.publish(poses);
        br.sendTransform(transforms);
    }

  private:
    ros::Publisher pub_;
    ros::Publisher publish_pointcloud_;
    std::vector<Eigen::Isometry3d> possible_transforms;
    tf2_ros::StaticTransformBroadcaster br;
    bool grasp_pose_received;
}; // End of class SubscribeAndPublisL

int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_broadcaster");
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::NodeHandle nh;
    param incoming_poses("gpd_interaction/estimated_poses", "");
    param outgoing_poses("gpd_interaction/outgoing_poses", "");
    readParameters(nh, incoming_poses, outgoing_poses);
    SubscribeAndPublish interaction(nh, outgoing_poses.second);
    ros::Subscriber sub =
        nh.subscribe(incoming_poses.second, QUEUE,
                     &SubscribeAndPublish::callback_object_pose, &interaction);
    ros::Subscriber sub2 =
        nh.subscribe("/detect_grasps/clustered_grasps", QUEUE,
                     &SubscribeAndPublish::callback_grasp_pose, &interaction);
    ros::waitForShutdown();
    return 0;
};
