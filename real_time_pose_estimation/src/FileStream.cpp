#include "FileStream.hpp"

#include <iostream>
#include <opencv2/imgcodecs.hpp>
namespace fs = std::filesystem;

void FileStream::open_folder(const DetectionParameters& paras) {
    fs::path path;
    if (fs::exists(paras.load_files)) {
        path = paras.load_files;
    } else {
        std::cout << "Cannot open path: " << paras.load_files << std::endl;
        throw std::runtime_error(" ");
    }
    iter = fs::begin(fs::directory_iterator(path));
    end = fs::end(fs::directory_iterator(path));
}

FileStream::FileStream(const DetectionParameters& paras) { open_folder(paras); }

bool FileStream::read(cv::Mat& img) {
    if (iter != end) {
        fs::path tmp = *iter;
        std::cout << "The filename is: " << tmp << std::endl;
        img = cv::imread(tmp);
        iter++;
        return true;
    } else {
        return false;
    }
}
