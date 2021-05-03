//#include "cluster.hpp"
#include "clustering.hpp"
//#include "segmentation.hpp"
//#include <chrono>
//#include <object_pose/positions.h>
//#include <pcl/common/centroid.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/search/search.h>
#include <ros/ros.h>
//#include <tf2_ros/transform_broadcaster.h>
#include <utils/utils.hpp>
#include <vector>
using param = std::pair<std::string, std::string>;
static constexpr std::size_t QUEUE(10);

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    param _topic("/cluster/topic", "");
    param segmentedTopic("/cluster/segmented", "");
    param estimated_poses("/cluster/centroids", "");
    utils::readParameters(nh, _topic, segmentedTopic, estimated_poses);
    Clustering::Clustering clustering(segmentedTopic.second, estimated_poses.second,
            nh);
    ros::Subscriber sub =
        nh.subscribe(_topic.second, QUEUE, &Clustering::Clustering::callback, &clustering);
    ros::spin();
    return 0;
}
