#ifndef detection_parse_parameters_hpp
#define detection_parse_parameters_hpp
#include <string>
#include <filesystem>
struct DetectionParameters{
    std::string video_read_path;
    std::string yml_read_path;
    std::string ply_read_path;
    int numKeyPoints;
    float ratioTest;
    int fast_match;
    int iterationsCount;
    float reprojectionError;
    float confidence;
    int minInliersKalman;
    int pnpMethod;
    std::string featureName;
    int useFLANN;
    std::string saveDirectory;
    int displayFilteredPose;
    std::string stream;
};

struct CameraParameters{
    double fx;
    double fy;
    double cx;
    double cy;
};

const DetectionParameters readDetectionParameters(const std::filesystem::path&);
const CameraParameters readCameraParameters(const std::filesystem::path&);
void displayParameters(const DetectionParameters&);
void displayCamera(const CameraParameters&);
#endif
