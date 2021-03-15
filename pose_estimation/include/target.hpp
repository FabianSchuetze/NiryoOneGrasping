#ifndef target_hpp
#define target_hpp
#include "model.hpp"
#include <filesystem>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <vector>

class Target : public Model {
  public:
    Target(const std::filesystem::path &, const std::filesystem::path &);
    ~Target() override = default;
};
#endif
