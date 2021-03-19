#ifndef target_hpp
#define target_hpp
#include "model.hpp"
#include <filesystem>
namespace fs = std::filesystem;
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <vector>

class Target : public Model {
  public:
    Target(const fs::path &, const fs::path &);
    ~Target() override = default;
};
#endif
