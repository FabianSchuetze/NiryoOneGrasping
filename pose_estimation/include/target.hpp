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
    std::string name() {return name_;}
  private:
    std::string name_;
};
#endif
