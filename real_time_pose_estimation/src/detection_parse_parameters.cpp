#include "detection_parse_parameters.hpp"

#include <iostream>
#include <opencv2/core.hpp>
namespace fs = std::filesystem;

template <typename T>
void getParameter(T& para, const std::string& name, const cv::FileStorage fs) {
    cv::FileNode n = fs[name];
    if (n.empty()) {
        std::cerr << "Could not open" << name << std::endl;
        throw std::runtime_error(" ");
    }
    para = static_cast<T>(n);
    //para = static_cast<T>(fs[name]);
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

const DetectionParameters readDetectionParameters(
    const fs::path& yml_location) {
    const cv::FileStorage storage = openStorage(yml_location);
    DetectionParameters paras;
    paras.video_read_path = fs::path((std::string)storage["video_read_path"]);
    paras.yml_read_path = fs::path((std::string)storage["yml_read_path"]);
    paras.ply_read_path = fs::path((std::string)storage["ply_read_path"]);
    getParameter(paras.ratioTest, "ratioTest", storage);
    getParameter(paras.iterationsCount, "iterationsCount", storage);
    getParameter(paras.numKeyPoints, "numKeyPoints", storage);
    getParameter(paras.fast_match, "fast_match", storage);
    getParameter(paras.reprojectionError, "reprojectionError", storage);
    getParameter(paras.confidence, "confidence", storage);
    getParameter(paras.minInliersKalman, "minInliersKalman", storage);
    getParameter(paras.pnpMethod, "pnpMethod", storage);
    getParameter(paras.featureName, "featureName", storage);
    getParameter(paras.useFLANN, "useFLANN", storage);
    getParameter(paras.saveDirectory, "saveDirectory", storage);
    getParameter(paras.displayFilteredPose, "displayFilteredPose", storage);
    return paras;
}

const CameraParameters readCameraParameters(const fs::path& camera_location) {
    const cv::FileStorage storage = openStorage(camera_location);
    CameraParameters camera;
    camera.fx = (double)storage["fx"];
    camera.fy = (double)storage["fy"];
    camera.cx = (double)storage["cx"];
    camera.cy = (double)storage["cy"];
    return camera;
}

void displayParameters(const DetectionParameters& paras) {
    std::cout << "Video: " << paras.video_read_path << std::endl;
    std::cout << "Training data: " << paras.yml_read_path << std::endl;
    std::cout << "CAD model: " << paras.ply_read_path << std::endl;
    std::cout << "Ratio test threshold: " << paras.ratioTest << std::endl;
    std::cout << "Fast match(no symmetry test)?: " << paras.fast_match
              << std::endl;
    std::cout << "RANSAC number of iterations: " << paras.iterationsCount
              << std::endl;
    std::cout << "RANSAC reprojection error: " << paras.reprojectionError
              << std::endl;
    std::cout << "RANSAC confidence threshold: " << paras.confidence
              << std::endl;
    std::cout << "Kalman number of inliers: " << paras.minInliersKalman
              << std::endl;
    std::cout << "PnP method: " << paras.pnpMethod << std::endl;
    std::cout << "Feature: " << paras.featureName << std::endl;
    std::cout << "Number of keypoints for ORB: " << paras.numKeyPoints
              << std::endl;
    std::cout << "Use FLANN-based matching? " << paras.useFLANN << std::endl;
    std::cout << "Save directory: " << paras.saveDirectory << std::endl;
    std::cout << "Display filtered pose from Kalman filter? "
              << paras.displayFilteredPose << std::endl;
}

void displayCamera(const CameraParameters& paras) {
    std::cout << "fx: " << paras.fx << std::endl;
    std::cout << "fy: " << paras.fy << std::endl;
    std::cout << "cx: " << paras.cx << std::endl;
    std::cout << "cy: " << paras.cy << std::endl;
}
