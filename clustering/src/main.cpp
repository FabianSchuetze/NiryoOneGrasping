#include "cluster.hpp"
#include "scene.hpp"
#include "segmentation.hpp"
//#include <chrono>
#include <object_pose/positions.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <utils/utils.hpp>
#include <vector>
using param = std::pair<std::string, std::string>;

using namespace Clustering;

static constexpr std::size_t QUEUE(10);
static constexpr std::size_t RATE(1);
// TODO: For testing experiments larger than I think is actually possible
static constexpr float RADIUS(0.45);
static constexpr float MIN_DISTANCE(0.1);
static constexpr std::size_t MIN_POINTS(300);
static constexpr std::size_t MAX_POINTS(50000);
static constexpr float MIN_RADIUS(0.02);
static constexpr std::size_t MAX_ITERATION(1000);
// TODO: Check if DISTANCE can be equal to MIN_RADIUS
static constexpr float DISTANCE(0.01);

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// template <typename... T>
// void readParameters(const ros::NodeHandle &nh, T &... args) {
// auto read_parameters = [&](auto &t) {
// nh.getParam(t.first, t.second);
// if (t.second.empty()) {
// ROS_WARN_STREAM("Rosparam " << t.first << " not identified");
// throw std::runtime_error("Could not read all parameters");
//}
// ROS_WARN_STREAM("The parameters for " << t.first << " is " << t.second);
//};
//(..., read_parameters(args));
//}

bool extractWorkspace(const PointCloud::ConstPtr &cloud,
                      PointCloud::Ptr &cluster) {
    pcl::KdTreeFLANN<pcl::PointXYZRGB> search;
    search.setInputCloud(cloud);
    pcl::PointXYZRGB origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    std::vector<float> distances;
    std::vector<int> indices;
    search.radiusSearch(origin, RADIUS, indices, distances);
    ROS_DEBUG_STREAM("The number of remaining points: " << indices.size());
    cluster->clear();
    for (int idx : indices) {
        auto pt = (*cloud)[idx];
        if (std::abs(pt.x) > MIN_DISTANCE) {
            (*cluster).push_back(pt);
        }
    }
    return !(cluster->empty());
}

bool segmentFloor(PlaneSegmentation<pcl::PointXYZRGB> &segmentation,
                  const PointCloud::ConstPtr &source, PointCloud::Ptr &target) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    segmentation.segment(inliers);
    bool setNegative = true;
    target->clear();
    extractPlane<pcl::PointXYZRGB>(source, target, inliers, setNegative);
    return !(target->empty());
}

pcl::PointXYZRGB centroid(const PointCloud::ConstPtr &input) {
    pcl::CentroidPoint<pcl::PointXYZRGB> center;
    for (const auto p : *input) {
        center.add(p);
    }
    pcl::PointXYZRGB out;
    center.get(out);
    // std::cout << out.x << ", " << out.y << ", " << out.z << std::endl;
    return out;
}

// TODO: I do not think this is needed anymore, as the  workspace function
// restricts the size
bool validCenter(const pcl::PointXYZRGB &point) {
    auto squared =
        static_cast<float>(std::pow(point.x, 2) + std::pow(point.y, 2));
    float dist = std::sqrt(squared);
    if (dist < 0.12) {
        std::cout << "Check is this a needed at all!" << std::endl;
        return false;
    }
    return true;
}

geometry_msgs::Pose convertToPose(const pcl::PointXYZRGB &centroid) {
    geometry_msgs::Pose pose;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    //geometry_msgs::Quaternion quat;
    pose.position.x = centroid.x;
    pose.position.y = centroid.y;
    pose.position.z = centroid.z;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

void broadcastClusters(const std::vector<PointCloud::Ptr> &clusters,
                       ros::Publisher &publisher, pcl::PCDWriter &writer) {
    object_pose::positions positions;
    std_msgs::Header header;
    header.frame_id = "base_link";
    header.seq = 0;
    positions.poses.header = header;
    int j(0);
    for (auto const &cluster : clusters) {
        auto point = centroid(cluster);
        if (validCenter(point)) {
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZRGB>(ss.str(), *cluster, false);
            geometry_msgs::Pose pose = convertToPose(point);
            std::string name = "object" + std::to_string(j);
            positions.objects.push_back(name);
            positions.poses.poses.push_back(pose);
            ++j;
        }
    }
    publisher.publish(positions);
}

// TODO: Should be written as a callback
int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    Scene scene;
    ros::Rate rate(1);
    param _topic("/cluster/topic", "");
    param segmentedTopic("/cluster/segmented", "");
    param estimated_poses("/cluster/centroids", "");
    utils::readParameters(nh, _topic, segmentedTopic, estimated_poses);
    ros::Subscriber sub =
        nh.subscribe(_topic.second, QUEUE, &Scene::callback, &scene);
    ros::Publisher publishSegmentation =
        nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(segmentedTopic.second,
                                                        QUEUE, true);
    ros::Publisher clusterPublisher =
        nh.advertise<object_pose::positions>(estimated_poses.second, 1, true);
    PointCloud::Ptr cloud(new PointCloud), workspace(new PointCloud),
        segmented(new PointCloud);
    ClusterAlgorithm<pcl::PointXYZRGB> cluster_algo(MIN_POINTS, MAX_POINTS,
                                                    MIN_RADIUS);
    PlaneSegmentation<pcl::PointXYZRGB> segmentation(MAX_ITERATION, DISTANCE);
    pcl::PCDWriter writer;
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        // ROS_WARN_STREAM("Inside here");
        if ((!scene.pointCloud(cloud)) or (cloud->empty())) {
            ROS_DEBUG_STREAM("No valid data arrived");
            continue;
        }
        writer.write<pcl::PointXYZRGB>("input_cloud.pcd", *cloud, false);
        if (!extractWorkspace(cloud, workspace)) {
            ROS_WARN_STREAM("The workspace has zero elements");
            continue;
        }
        writer.write<pcl::PointXYZRGB>("workspace.pcd", *workspace, false);
        segmentation.setInputCloud(workspace);
        if (!segmentFloor(segmentation, workspace, segmented)) {
            ROS_WARN_STREAM("No points remain after segmentation");
            continue;
        }
        // writer.write<pcl::PointXYZRGB>("segmented.pcd", *segmented, false);
        std::vector<PointCloud::Ptr> clusters{};
        cluster_algo.cluster(segmented, clusters);
        segmented->header.frame_id = "base_link";
        publishSegmentation.publish(*segmented);
        ros::spinOnce();
        if (clusters.empty()) {
            ROS_WARN_STREAM("Could not find any clusters");
            continue;
        }
        broadcastClusters(clusters, clusterPublisher, writer);
    }
    sub.shutdown();
    return 0;
}
