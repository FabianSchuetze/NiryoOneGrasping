#include "cluster.hpp"

#include <pcl/search/kdtree.h>

#include <algorithm>
template <typename T>
ClusterAlgorithm<T>::ClusterAlgorithm(std::size_t min_points,
                                      std::size_t max_points, float tolerance) {
    tree = std::make_unique<pcl::search::KdTree<T>>();
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_points);
    ec.setMaxClusterSize(max_points);
    ec.setSearchMethod(tree);
}
template class ClusterAlgorithm<pcl::PointXYZ>;
template class ClusterAlgorithm<pcl::PointXYZRGB>;

template <typename T>
void ClusterAlgorithm<T>::cluster(
    const typename pcl::PointCloud<T>::ConstPtr cloud,
    typename std::vector<typename pcl::PointCloud<T>::Ptr>& clusters) {
    ec.setInputCloud(cloud);
    std::vector<pcl::PointIndices> indices;
    ec.extract(indices);
    for (const auto& index : indices) {
        typename pcl::PointCloud<T>::Ptr cluster(new pcl::PointCloud<T>);
        cluster->reserve(index.indices.size());
        for (auto idx : index.indices) {
            auto tmp = (*cloud)[idx];
            (*cluster).push_back(tmp);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }
}
