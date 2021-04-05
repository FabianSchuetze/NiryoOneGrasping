#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/file_io.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_pcl");
    pcl::PCDReader reader{};
    PointCloud::Ptr cloud( new PointCloud);
    reader.read("/home/fabian/Documents/work/transforms/src/ObjectPose/data/"
                "integrated.pcd",
                *cloud);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("reconstructed_scene", 1);
    ros::Rate loop_rate(4);
    while (nh.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        pub.publish(cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
