#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

geometry_msgs::TransformStamped
generateTransformation(const geometry_msgs::Pose &pose, double yaw) {
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
    return transformStamped;
}

geometry_msgs::Pose generateGraspPose(const geometry_msgs::Pose &pose,
                                      double yaw) {
    geometry_msgs::Pose grasp_pose;
    grasp_pose.position = pose.position;
    grasp_pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    grasp_pose.orientation = tf2::toMsg(q);
    return grasp_pose;
}

class SubscribeAndPublish {
  public:
    SubscribeAndPublish(ros::NodeHandle &n_, const std::string &publication)
        {
        ROS_WARN_STREAM("The publication is" << publication);
        pub_ = n_.advertise<geometry_msgs::PoseArray>(publication, 1, true);
    }
    void callback(const geometry_msgs::PoseArray &msg) {
        ROS_WARN_STREAM("Inside the callback");
        int i(0);
        const auto now = ros::Time::now();
        std::vector<geometry_msgs::TransformStamped> transforms;
        geometry_msgs::PoseArray poses;
        poses.header.frame_id = msg.header.frame_id;
        poses.header.stamp = now;
        for (auto const &pose : msg.poses) {
            double yaw = obtain_yaw(pose.orientation);
            auto transformStamped = generateTransformation(pose, yaw);
            auto grasp_pose = generateGraspPose(pose, yaw);
            transformStamped.header.stamp = now;
            transformStamped.header.frame_id = msg.header.frame_id;
            transformStamped.child_frame_id = "/object_" + std::to_string(i);
            ROS_WARN_STREAM("The child_frame_id is: " << transformStamped.child_frame_id);
            poses.poses.push_back(grasp_pose);
            transforms.push_back(transformStamped);
            ++i;
        }
        ROS_WARN_STREAM("Transfroms size: " << transforms.size());
        pub_.publish(poses);
        br.sendTransform(transforms);
        ROS_WARN_STREAM("At the end");
    }

  private:
    ros::Publisher pub_;
    tf2_ros::StaticTransformBroadcaster br;

}; // End of class SubscribeAndPublisL

int main(int argc, char **argv) {
    // ros::Publisher publisher;
    ros::init(argc, argv, "grasp_broadcaster");
    ros::NodeHandle nh;
    param incoming_poses("grasp_pose_broadcaster/estimated_poses", "");
    param outgoing_poses("grasp_pose_broadcaster/outgoing_poses", "");
    readParameters(nh, incoming_poses, outgoing_poses);
    SubscribeAndPublish interaction(nh, outgoing_poses.second);
    ros::Subscriber sub =
        nh.subscribe(incoming_poses.second, QUEUE,
                     &SubscribeAndPublish::callback, &interaction);
    ros::spin();
    return 0;
};
