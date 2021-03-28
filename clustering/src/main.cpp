#include "cluster.hpp"
#include "scene.hpp"
#include "segmentation.hpp"
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <chrono>

static constexpr std::size_t QUEUE(10);
static constexpr std::size_t RATE(1);
static constexpr float RADIUS(0.25);

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void extractWorkspace(PointCloud::ConstPtr cloud, PointCloud::Ptr &cluster) {
    pcl::KdTreeFLANN<pcl::PointXYZRGB> search;
    search.setInputCloud(cloud);
    pcl::PointXYZRGB origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    std::vector<float> distances;
    std::vector<int> indices;
    cluster->reserve(indices.size());
    search.radiusSearch(origin, RADIUS, indices, distances);
    std::transform(indices.begin(), indices.end(), cluster->begin(),
                   [&](int idx) { return (*cloud)[idx]; });
}

void segmentFloor(PlaneSegmentation<pcl::PointXYZRGB> &segmentation,
                  const PointCloud::ConstPtr &source, PointCloud::Ptr &target) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    segmentation.segment(inliers);
    bool setNegative = true;
    extractPlane<pcl::PointXYZRGB>(source, target, inliers, setNegative);
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

bool validCenter(const pcl::PointXYZRGB &point) {
    float squaredDist = std::pow(point.x, 2) + std::pow(point.y, 2);
    float dist = std::sqrt(squaredDist);
    if (dist < 0.12) {
        return false;
    }
    return true;
}

void broadcastPosition(const pcl::PointXYZRGB &point) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "object";
    transformStamped.transform.translation.x = point.x;
    transformStamped.transform.translation.y = point.y;
    transformStamped.transform.translation.z = point.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    Scene scene;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", QUEUE,
                                       &Scene::callback, &scene);
    PointCloud::Ptr cloud(new PointCloud), workspace(new PointCloud),
        segmented(new PointCloud);
    ClusterAlgorithm<pcl::PointXYZRGB> cluster_algo(300, 25000, 0.02);
    PlaneSegmentation<pcl::PointXYZRGB> segmentation(1000, 0.01);
    pcl::PCDWriter writer;
    while (ros::ok()) {
        const auto t1 = std::chrono::high_resolution_clock::now();
        ros::spinOnce();
        if (!scene.pointCloud(cloud)) {
            continue;
        }
        if (cloud->empty()) {
            ROS_WARN_STREAM("Empty cloud");
            continue;
        }
        extractWorkspace(cloud, workspace);
        writer.write<pcl::PointXYZRGB>("workspace.pcd", *workspace, false);
        segmentation.setInputCloud(workspace);
        segmentFloor(segmentation, workspace, segmented);
        writer.write<pcl::PointXYZRGB>("segmented.pcd", *segmented, false);
        std::vector<PointCloud::Ptr> clusters;
        cluster_algo.cluster(segmented, clusters);
        if (clusters.empty()) {
            ROS_WARN_STREAM("Could not find any clusters");
        }
        int j(0);
        for (const auto &cluster : clusters) {
            auto point = centroid(cluster);
            if (validCenter(point)) {
                std::cout << "Found a valid point at " << point.x << ", "
                          << point.y << ", " << point.z << std::endl;
                broadcastPosition(point);
                std::stringstream ss;
                ss << "cloud_cluster_" << j << ".pcd";
                writer.write<pcl::PointXYZRGB>(ss.str(), *cluster, false);
                ++j;
            }
        }
        const auto t2 = std::chrono::high_resolution_clock::now();
        const auto dur = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1);
        std::cout << "The run took" << dur.count() << std::endl;
    }
    sub.shutdown();
    return 0;
}
