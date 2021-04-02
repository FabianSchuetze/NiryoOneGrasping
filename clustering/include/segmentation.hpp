#ifndef segmentation_hpp
#define segmentation_hpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

template <typename T>
class PlaneSegmentation {
   public:
    PlaneSegmentation(std::size_t, float);
    void setInputCloud(const typename pcl::PointCloud<T>::ConstPtr&);
    bool segment(pcl::PointIndices::Ptr&);

   private:
    pcl::SACSegmentation<T> seg;
};

template <typename T>
void extractPlane(const typename pcl::PointCloud<T>::ConstPtr&,
                  typename pcl::PointCloud<T>::Ptr&, pcl::PointIndices::Ptr,
                  bool);
#endif
