#include <chrono>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <thread>
#include <mutex>
static constexpr std::size_t INIT_TIME(5);
class Scene {
  public:
    Scene()
        : cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
          last_callback(std::chrono::system_clock::now()) {
        // last_callback = std::chrono::system_clock::now();
        std::this_thread::sleep_for(std::chrono::seconds(INIT_TIME));
    }
    ~Scene() = default;
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
    bool pointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &);

  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    tf::TransformListener listener;
    std::chrono::time_point<std::chrono::system_clock> last_callback;
    std::mutex mutex_;
};
