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
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
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

double obtain_yaw(const geometry_msgs::Quaternion &quat) {
    double first = 2 * (quat.w * quat.z + quat.x * quat.y);
    double second = 1 - 2 * (quat.x * quat.x + quat.z * quat.z);
    double yaw = std::atan2(first, second);
    // if (yaw > PI / 2.0) {
    // yaw = yaw - PI;
    //} else if (yaw < -PI / 2.0) {
    // yaw = yaw + PI;
    //} // angles are close to zero;
    return yaw;
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

    void callback_grasp_pose(const gpd_ros::GraspConfigList &msg) {
        ROS_WARN_STREAM("Inside the grap pose callback");
        const std::string name = msg.header.frame_id;
        const gpd_ros::GraspConfig &grasp = msg.grasps[0];
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
        ros_transform.matrix() = frame;
        grasp_pose_received = true;
    }

    pcl::PointXYZ inline toPointXYZ(const Eigen::Vector3d &point) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>(point(0));
        pcl_point.y = static_cast<float>(point(1));
        pcl_point.z = static_cast<float>(point(2));
        // pcl_point.r = static_cast<uint8_t>(std::round(color(0) * MAX_UINT));
        // pcl_point.g = static_cast<uint8_t>(std::round(color(1) * MAX_UINT));
        // pcl_point.b = static_cast<uint8_t>(std::round(color(2) * MAX_UINT));
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
        const std::vector<Eigen::Vector3d> &points = cloud->points_;
        const std::vector<Eigen::Vector3d> &colors = cloud->colors_;
        for (std::size_t idx = 0; idx < points.size(); ++idx) {
            auto point = toPointXYZ(points[idx]);
            pcl_cloud->push_back(point);
        }
        pcl_cloud->height = 1;
        pcl_cloud->width = points.size();
        std::string fn = fs::path(name).filename();
        pcl_cloud->header.frame_id = fn;
        publish_pointcloud_.publish(pcl_cloud);
    }

    void callback_object_pose(const object_pose::positions &msg) {
        grasp_pose_received = false;
        ROS_WARN_STREAM("Inside the object pose callback");
        const auto now = ros::Time::now();
        std::vector<geometry_msgs::TransformStamped> transforms;
        geometry_msgs::PoseArray poses;
        poses.header.frame_id = msg.poses.header.frame_id;
        poses.header.stamp = now;
        for (std::size_t i = 0; i < msg.objects.size(); ++i) {
            grasp_pose_received = false;
            geometry_msgs::Pose pose = msg.poses.poses[i];
            const std::string &name = msg.objects[i];
            double yaw = obtain_yaw(pose.orientation);
            Eigen::Isometry3d transformStamped =
                generateTransformation(pose, yaw);
            auto transform = tf2::eigenToTransform(transformStamped);
            transform.header.frame_id = "base_link";
            transform.child_frame_id = shortName(name, "");
            publishPointCloud(name);
            while ((!grasp_pose_received) and ros::ok()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            grasp_pose_received = false;
            Eigen::Isometry3d tmp = transformStamped * ros_transform;
            auto final_transform = tf2::eigenToTransform(tmp);
            final_transform.child_frame_id = shortName(name, "_grasp");
            final_transform.header.frame_id = "base_link";
            transforms.push_back(transform);
            transforms.push_back(final_transform);
        }
        ROS_WARN_STREAM("Transfroms size: " << transforms.size());
        pub_.publish(poses);
        br.sendTransform(transforms);
    }

  private:
    ros::Publisher pub_;
    ros::Publisher publish_pointcloud_;
    Eigen::Isometry3d ros_transform;
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
