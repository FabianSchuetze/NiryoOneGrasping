#ifndef detection_parse_parameters_hpp
#define detection_parse_parameters_hpp
#include <string>
#include <filesystem>
struct DetectionParameters{
    std::string video_read_path;
    std::string yml_read_path;
    std::string ply_read_path;
    std::string save_storage;
    std::string load_files;
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

struct KalmanFilterParameters{
    int nStates;       // the number of states
    int nMeasurements;  // the number of measured states
    int nInputs;        // the number of control actions
    double dt;      // time between measurements (1/FPS)
};

const DetectionParameters readDetectionParameters(const std::filesystem::path&);
const CameraParameters readCameraParameters(const std::filesystem::path&);
const KalmanFilterParameters readKalmanFilterParameters(const std::filesystem::path&);
void displayParameters(const DetectionParameters&);
void displayCamera(const CameraParameters&);
void displayKalman(const KalmanFilterParameters&);
#endif
