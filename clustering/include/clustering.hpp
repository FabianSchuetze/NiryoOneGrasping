#ifndef clustering_scene_hpp
#define clustering_scene_hpp
#include <chrono>
#include <mutex>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <thread>
#include "cluster.hpp"
#include "segmentation.hpp"
static constexpr std::size_t INIT_TIME(5);
namespace Clustering {
class Clustering {
  public:
    Clustering(const std::string&, const std::string&, ros::NodeHandle&);
        //: last_callback(0, 0) {
        // last_callback = std::chrono::system_clock::now();
        //std::this_thread::sleep_for(std::chrono::seconds(INIT_TIME));
    //}
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
    bool extractFrame(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &);
    void extractInfo(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);


  private:
    void broadcastClusters(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    tf::TransformListener listener;
    ros::Time last_callback;
    ClusterAlgorithm<pcl::PointXYZRGB> cluster_algo;
    PlaneSegmentation<pcl::PointXYZRGB> segmentation;
    pcl::PCDWriter writer;
    ros::Publisher publishSegmentation, publishCluster;
    std::size_t iteration;
    // std::mutex mutex_;
};
} // namespace Clustering
#endif
