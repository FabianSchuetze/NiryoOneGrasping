#include "segmentation.hpp"
#include <pcl/filters/extract_indices.h>

template <typename T>
PlaneSegmentation<T>::PlaneSegmentation(
    std::size_t max_iterations, float distance,
    typename pcl::PointCloud<T>::ConstPtr cloud) {
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance);
    seg.setInputCloud(cloud);
}

template class PlaneSegmentation<pcl::PointXYZ>;
template class PlaneSegmentation<pcl::PointXYZRGB>;

template <typename T>
bool PlaneSegmentation<T>::segment(pcl::PointIndices::Ptr inliers) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset."
                  << std::endl;
        return false;
    }
    return true;
}

template <typename T>
void extractPlane(typename pcl::PointCloud<T>::ConstPtr source,
                  typename pcl::PointCloud<T>::Ptr target,
                  pcl::PointIndices::Ptr inliers, bool setNegative) {
    pcl::ExtractIndices<T> extract;
    extract.setInputCloud(source);
    extract.setIndices(inliers);
    extract.setNegative(setNegative);
    extract.filter(*target);
}

template void extractPlane<pcl::PointXYZ>(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointIndices::Ptr, bool);
template void extractPlane<pcl::PointXYZRGB>(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointIndices::Ptr, bool);
