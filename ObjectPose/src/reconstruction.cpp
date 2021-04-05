#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/io/file_io.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

template <typename M> M load_csv(const std::string &path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ' ')) {
            try {
                values.push_back(std::stod(cell));
            } catch (const std::invalid_argument &e) {
                ;
            }
        }
        ++rows;
    }
    if (rows > 0) {
        return Eigen::Map<
            const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime,
                                M::ColsAtCompileTime, Eigen::RowMajor>>(
            values.data(), rows, values.size() / rows);
    }
    throw std::runtime_error("Could not read input file");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pub_pcl");
    pcl::PCDReader reader{};
    PointCloud::Ptr source(new PointCloud), transformed(new PointCloud);
    reader.read("/home/fabian/Documents/work/transforms/src/ObjectPose/data/"
                "integrated.pcd",
                *source);
    //Eigen::Matrix4Xd mat = load_csv<Eigen::Matrix4Xd>(
        //"/home/fabian/2021-04-03-16-39/transform_pth/000.txt");
    //std::cout << mat << std::endl;
    Eigen::Affine3d transform;
    transform.matrix() = load_csv<Eigen::Matrix4d>(
        "/home/fabian/2021-04-03-16-39/transform_pth/000.txt");
    pcl::transformPointCloud(*source, *transformed, transform.inverse());
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("reconstructed_scene", 1);
    ros::Rate loop_rate(4);
    while (nh.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), transformed->header.stamp);
        transformed->header.frame_id = "base_link";
        pub.publish(transformed);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
