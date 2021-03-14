#include "target.hpp"
#include <opencv2/core.hpp>
#include <iostream>
#include <type_traits>
namespace fs = std::filesystem;

template <typename T>
void getParameter(T& para, const std::string& name, const cv::FileStorage fs) {
    cv::FileNode n = fs[name];
    if (n.empty()) {
        std::cerr << "Could not open" << name << std::endl;
        throw std::runtime_error(" ");
    }
    if constexpr (std::is_same_v<T, cv::Mat>) {
        fs[name] >> para;
    } else {
        para = static_cast<T>(n);
    }
}

const cv::FileStorage openStorage(const fs::path& path) {
    bool exists = fs::exists(path);
    if (!exists) {
        std::cerr << "File does not exists " << path << std::endl;
        throw std::runtime_error("");
    }
    cv::FileStorage storage;
    storage.open(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        throw std::runtime_error("Could not open filestorage");
    }
    return storage;
}
Target::Target(const fs::path& _model_description,
               const fs::path& _img_location) {
    cv::FileStorage fs = openStorage(_model_description);
    getParameter(descriptors_, "descriptors", fs);
    getParameter(points3d, "points3d", fs);
    //getParameter(points2d, "points2d", fs);
    cv::read(fs["keypoints"], kps);
    //descriptors = fs["descriptors"].mat();


}
