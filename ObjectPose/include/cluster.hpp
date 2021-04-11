#ifndef cluster_hpp
#define cluster_hpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
template <typename T>
class ClusterAlgorithm {
   public:
    ClusterAlgorithm(std::size_t, std::size_t, float);
    void cluster(const typename pcl::PointCloud<T>::ConstPtr,
                 typename std::vector<typename pcl::PointCloud<T>::Ptr>&);

   private:
    typename pcl::EuclideanClusterExtraction<T> ec;
    typename pcl::search::KdTree<T>::Ptr tree;
};
#endif
