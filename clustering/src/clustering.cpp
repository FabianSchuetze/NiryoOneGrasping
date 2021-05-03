#include "clustering.hpp"
#include <object_pose/positions.h>
#include <pcl_ros/transforms.h>

static std::string BASE_LINK("base_link");
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static constexpr float RADIUS(0.45);
static constexpr float MIN_DISTANCE(0.1);
static constexpr std::size_t MIN_POINTS(300);
static constexpr std::size_t MAX_POINTS(50000);
static constexpr float MIN_RADIUS(0.02);
static constexpr std::size_t MAX_ITERATION(1000);
static constexpr float DISTANCE(0.01);
namespace Clustering {

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
    return out;
}

// iODO: I do not think this is needed anymore, as the  workspace function
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
    pose.position.x = centroid.x;
    pose.position.y = centroid.y;
    pose.position.z = centroid.z;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

void Clustering::broadcastClusters(
    const std::vector<PointCloud::Ptr> &clusters) {
    object_pose::positions positions;
    std_msgs::Header header;
    header.frame_id = "base_link";
    header.seq = iteration;
    header.stamp = ros::Time::now();
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
    publishCluster.publish(positions);
    ++iteration;
}

bool Clustering::extractFrame(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out) {
    std::string frame = input->header.frame_id;
    ros::Time send_time;
    pcl_conversions::fromPCL(input->header.stamp, send_time);
    if (send_time < last_callback) {
        return false;
    }
    last_callback = ros::Time::now();
    if (!(frame == BASE_LINK)) {
        const std::string target("base_link");
        if (!pcl_ros::transformPointCloud(target, *input, *out, listener)) {
            ROS_WARN_STREAM("Cannot transform pointcloud");
            return false;
        }
    } else {
        *out = *input;
    }
    return true;
}

Clustering::Clustering(const std::string &segmentedTopic,
                       const std::string &estimated_poses, ros::NodeHandle &nh)
    : last_callback(0, 0), cluster_algo(MIN_POINTS, MAX_POINTS, MIN_RADIUS),
      segmentation(MAX_ITERATION, DISTANCE), iteration(0) {
    publishSegmentation = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
        segmentedTopic, 1, true);
    publishCluster =
        nh.advertise<object_pose::positions>(estimated_poses, 1, true);
}

void Clustering::extractInfo(const PointCloud::Ptr &cloud) {
    writer.write<pcl::PointXYZRGB>("input_cloud.pcd", *cloud, false);
    PointCloud::Ptr workspace(new PointCloud), segmented(new PointCloud);
    if (!extractWorkspace(cloud, workspace)) {
        ROS_WARN_STREAM("The workspace has zero elements");
        return;
    }
    writer.write<pcl::PointXYZRGB>("workspace.pcd", *workspace, false);
    segmentation.setInputCloud(workspace);
    if (!segmentFloor(segmentation, workspace, segmented)) {
        ROS_WARN_STREAM("No points remain after segmentation");
        return;
    }
    writer.write<pcl::PointXYZRGB>("segmented.pcd", *segmented, false);
    std::vector<PointCloud::Ptr> clusters{};
    cluster_algo.cluster(segmented, clusters);
    segmented->header.frame_id = "base_link";
    publishSegmentation.publish(*segmented);
    ros::spinOnce();
    if (clusters.empty()) {
        ROS_WARN_STREAM("Could not find any clusters");
        return;
    }
    broadcastClusters(clusters);
}

void Clustering::callback(const PointCloud::ConstPtr &input) {
    PointCloud::Ptr cloud(new PointCloud);
    if (!extractFrame(input, cloud)) {
        return;
    }
    extractInfo(cloud);
}
} // namespace Clustering
