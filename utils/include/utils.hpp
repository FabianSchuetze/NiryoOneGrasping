//#ifndef utils_hpp
//#define utils_hpp
//#include <geometry_msgs/TransformStamped.h>
//#include <geometry_msgs/Pose.h>
//#include <ros/ros.h>
//namespace utils {
//template <typename... T>
//void readParameters(const ros::NodeHandle &nh, T &... args) {
    //auto read_parameters = [&](auto &t) {
        //nh.getParam(t.first, t.second);
        //if (t.second.empty()) {
            //ROS_ERROR_STREAM("Rosparam " << t.first << " not identified");
            //throw std::runtime_error("Could not read all parameters");
        //}
        //ROS_WARN_STREAM("The parameters for " << t.first << " is " << t.second);
    //};
    //(..., read_parameters(args));
//}
//class DOF{
    //public:
        ////explicit DOF(const geometry_msgs::Pose& pose);
        //DOF(double, double, double, double, double, double);
        //double x, y, z, roll, pitch, yaw;
//};
//geometry_msgs::TransformStamped generateTransformation(const DOF &);
//} // namespace utils
//#endif
