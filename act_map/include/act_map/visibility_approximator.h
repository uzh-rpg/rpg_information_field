#pragma once

#include <string>
#include <fstream>
#include <memory>

#include <Eigen/Core>

namespace act_map
{

template <class T>
class VisibilityApproximator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~VisibilityApproximator() {}

  bool isInitialized() const
  {
    return impl()->isInitialized();
  }

  void load(const std::string& folder)
  {
    impl()->load(folder);
  }

  void calLandmarkFeatureVector(const Eigen::Vector3d& fpt_w,
                                Eigen::VectorXd* lm_ftrs) const
  {
    impl()->calLandmarkFeatureVector(fpt_w, lm_ftrs);
  }

  void calCamZFeatureVector(const Eigen::Vector3d& camz_w,
                            Eigen::VectorXd* camz_ftrs) const
  {
    impl()->calCamZFeatureVector(camz_w, camz_ftrs);
  }

  int dimFeatureSpace() const
  {
    return impl()->dimFeatureSpace();
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const VisibilityApproximator& vis)
  {
    out << vis.impl()->str();
    return out;
  }

  inline T* impl()
  {
    return static_cast<T*>(this);
  }

  inline const T* impl() const
  {
    return static_cast<const T*>(this);
  }

private:

};

template <typename T>
using VisApproxPtr = std::shared_ptr<VisibilityApproximator<T>>;
}
