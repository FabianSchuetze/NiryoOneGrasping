#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

static constexpr uint QUEUE = 10;
static constexpr char listen[] = "/grasp_position"; // NOLINT

void poseCallback(const geometry_msgs::Point &msg) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_link";
    transformStamped.child_frame_id = "object";
    transformStamped.transform.translation.x = msg.x;
    transformStamped.transform.translation.y = msg.y;
    transformStamped.transform.translation.z = msg.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_object_broadcaster");
    ros::NodeHandle private_node("~");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(listen, QUEUE, &poseCallback);
    ros::spin();
    sub.shutdown();
    return 0;
};
