#pragma once

#include "act_map/info_utils.h"

#include "act_map/act_map.h"

#include <memory>
#include <random>

namespace act_map
{
struct InfoPotentialOptions
{
  double min_depth_m_ = 1;
  double max_depth_m_ = 3;
  double px_noise_sigma_px_ = 1.0;
  int n_random_landmarks_ = 50;
  double fov_deg_ = 90;

  int average_over_n_ = 50;

  double val_at_zero_ = 100.0;

  // simply take the negative information
  bool use_negative_info_ = false;
};

// general potential function
class InfoPotentialFunc
{
public:
  InfoPotentialFunc()
  {
  }
  InfoPotentialFunc(const InfoPotentialFunc& rhs);
  InfoPotentialFunc(const double thresh, const double val_at_zero);

  double eval(const double query, double* grad = nullptr) const;

private:
  double thresh_;
  double val_at_zero_;
  double quad_k_;

  double linear_k_;
  double linear_b_;
};

namespace traits
{
template <typename T>
struct ValidPotentialMetrics
{
  using type = std::array<InfoMetricType, 3>;
  constexpr static type values{ InfoMetricType::kDet, InfoMetricType::kTrace,
                                InfoMetricType::kMinEig };
};
template <typename T>
constexpr
    typename ValidPotentialMetrics<T>::type ValidPotentialMetrics<T>::values;

template <>
struct ValidPotentialMetrics<QuadTraceVoxel>
{
  using type = std::array<InfoMetricType, 1>;
  constexpr static type values{ InfoMetricType::kTrace };
};

template <>
struct ValidPotentialMetrics<GPTraceVoxel>
{
  using type = std::array<InfoMetricType, 1>;
  constexpr static type values{ InfoMetricType::kTrace };
};

}  // namespace traits

template <typename T> class InformationPotential;
template <typename T>
std::ostream& operator<<(std::ostream& os, const InformationPotential<T>& info);

template <typename T>
class InformationPotential
{
public:
  const static std::vector<InfoMetricType> kValidMetric;

  using InfoTThreshMap = std::unordered_map<InfoMetricType, double>;
  using InfoTFuncMap = std::unordered_map<InfoMetricType, InfoPotentialFunc>;

  InformationPotential() = delete;
  InformationPotential(const InformationPotential& rhs) = delete;

  InformationPotential(const InfoPotentialOptions& options,
                       const QuadVisScoreOptions& vis_options = QuadVisScoreOptions());

  inline bool isInitialized() const
  {
    return initialized_;
  }

  inline bool isMetricValid(const InfoMetricType& t) const
  {
    return (std::find(kValidMetric.begin(), kValidMetric.end(), t) !=
            kValidMetric.end());
  }

  inline double getMetricThresh(const InfoMetricType& info_t) const
  {
    CHECK(isMetricValid(info_t));
    return info_t_to_thresh_map_.at(info_t);
  }

  inline double getMetricThreshApproxVis(const InfoMetricType& info_t) const
  {
    CHECK(isMetricValid(info_t));
    return approx_info_t_to_thresh_map_.at(info_t);
  }

  inline double negInfo(const double x, double* grad = nullptr) const
  {
    double cost = -x;
    if (grad)
    {
      (*grad) = -1.0;
    }
    return cost;
  }

  inline double eval(const double x, const InfoMetricType& info_t,
                     double* grad = nullptr) const
  {
    if (options_.use_negative_info_)
    {
      return negInfo(x, grad);
    }
    else
    {
      return info_t_to_potential_func_map_.at(info_t).eval(x, grad);
    }
  }

  inline double evalApproxVis(const double x, const InfoMetricType& info_t,
                              double* grad = nullptr) const
  {
    if (options_.use_negative_info_)
    {
      return negInfo(x, grad);
    }
    else
    {
      return approx_info_t_to_potential_func_map_.at(info_t).eval(x, grad);
    }
  }

  inline void reinit(const InfoPotentialOptions& new_opts)
  {
    options_ = new_opts;
    info_t_to_thresh_map_.clear();
    info_t_to_potential_func_map_.clear();
    approx_info_t_to_thresh_map_.clear();
    approx_info_t_to_potential_func_map_.clear();
    init();
  }

  inline InfoPotentialOptions getOptionsCopy() const
  {
    return options_;
  }

  // clang-format off
  friend std::ostream& operator<< <>(std::ostream& os,
                                     const InformationPotential<T>& info_pot);
  // clang-format on

private:
  InfoPotentialOptions options_;
  void init();

  void initPotentialFuncApprox(const Vec3dVec& points_w_vec,
                               const rpg::Pose& Twc,
                               const ActMapOptions& am_opts,
                               const std::vector<double>& ranges,
                               InfoTThreshMap* thresh_map,
                               InfoTFuncMap* func_map);

  QuadVisScoreOptions quad_vis_options_;

  bool initialized_ = false;

  InfoTThreshMap info_t_to_thresh_map_;
  InfoTFuncMap info_t_to_potential_func_map_;

  InfoTThreshMap approx_info_t_to_thresh_map_;
  InfoTFuncMap approx_info_t_to_potential_func_map_;
};

template <typename T>
const std::vector<InfoMetricType> InformationPotential<T>::kValidMetric(
    traits::ValidPotentialMetrics<T>::values.begin(),
    traits::ValidPotentialMetrics<T>::values.end());

template <typename T>
using InfoPotentialPtr = std::shared_ptr<InformationPotential<T>>;

template <typename T>
InformationPotential<T>::InformationPotential(
    const InfoPotentialOptions& opts, const QuadVisScoreOptions& vis_score_opts)
  : options_(opts), quad_vis_options_(vis_score_opts)

{
  init();
}

template <typename T>
void InformationPotential<T>::init()
{
  info_t_to_thresh_map_.clear();
  info_t_to_potential_func_map_.clear();

  // simulate a camera observe landmarks
  const int total_pts = options_.n_random_landmarks_ * options_.average_over_n_;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(options_.min_depth_m_,
                                              options_.max_depth_m_);
  const double hfov_rad = options_.fov_deg_ * 0.5 * M_PI / 180.0;
  std::uniform_real_distribution<double> ang(-hfov_rad, hfov_rad);

  VLOG(1) << "Generating random points...";
  const Eigen::Vector3d z_ax(0.0, 0.0, 1.0);
  rpg::Pose Tcw;
  Tcw.setIdentity();
  const double pix_cov =
      options_.px_noise_sigma_px_ * options_.px_noise_sigma_px_;
  Vec3dVec points_w_vec;
  for (int i = 0; i < total_pts; i++)
  {
    Eigen::Vector3d raxis;
    raxis.setRandom();
    raxis.normalize();
    raxis *= ang(gen);
    rpg::Rotation rot(raxis);
    const Eigen::Vector3d p_c = rot.rotate(z_ax) * dist(gen);
    points_w_vec.emplace_back(Tcw.inverse().transform(p_c));
  }

  VLOG(1) << "===> Calculate from point cloud:" << std::endl;
  std::unordered_map<InfoMetricType, double> type_sum;
  for (const InfoMetricType& t : kValidMetric)
  {
    type_sum.insert({ t, 0.0 });
  }
  for (int ti = 0; ti < options_.average_over_n_; ti++)
  {
    int pt_start = ti * options_.n_random_landmarks_;
    rpg::Matrix66 info;
    info.setZero();
    for (int li = 0; li < options_.n_random_landmarks_; li++)
    {
      const Eigen::Vector3d p_w = points_w_vec[pt_start + li];
      rpg::Matrix36 dbearing_dpose =
          vi_utils::jacobians::dBearing_dIMUPoseGlobal(p_w, Tcw);
      info += dbearing_dpose.transpose() * dbearing_dpose * (1.0 / (pix_cov));
    }

    for (const InfoMetricType& t : kValidMetric)
    {
      double v = getInfoMetric(info, t);
      type_sum[t] += v;
    }
  }
  for (const InfoMetricType& t : kValidMetric)
  {
    info_t_to_thresh_map_.insert({ t, type_sum[t] / options_.average_over_n_ });
    info_t_to_potential_func_map_.insert(
        { t,
          InfoPotentialFunc(info_t_to_thresh_map_[t], options_.val_at_zero_) });
  }

  VLOG(1) << "===> Calculate from map:" << std::endl;
  ActMapOptions am_opts;
  am_opts.occ_layer_options_.vox_size = 0.01;
  am_opts.occ_layer_options_.vox_per_side = 2;
  am_opts.pos_factor_layer_options_.vox_size = 0.01;
  am_opts.pos_factor_layer_options_.vox_per_side = 2;
  std::vector<double> around_zero{ 0.05, 0.05, 0.05 };
  am_opts.vis_options_.clear();
  am_opts.vis_options_.push_back(quad_vis_options_);
  initPotentialFuncApprox(points_w_vec, Tcw.inverse(), am_opts, around_zero,
                          &approx_info_t_to_thresh_map_,
                          &approx_info_t_to_potential_func_map_);

  initialized_ = true;
}

template <typename T>
void InformationPotential<T>::initPotentialFuncApprox(
    const Vec3dVec& points_w_vec, const rpg::Pose& Twc,
    const ActMapOptions& am_opts, const std::vector<double>& ranges,
    InfoTThreshMap* thresh_map, InfoTFuncMap* func_map)
{
  thresh_map->clear();
  func_map->clear();
  std::unordered_map<InfoMetricType, double> type_sum;
  for (const InfoMetricType& t : kValidMetric)
  {
    type_sum.insert({ t, 0.0 });
  }
  ActMap<T> approx_info_map(am_opts);
  approx_info_map.allocateFactorLayerUniform(ranges);
  for (int ti = 0; ti < options_.average_over_n_; ti++)
  {
    approx_info_map.occLayerPtr()->removeAllBlocks();

    int pt_start = ti * options_.n_random_landmarks_;
    Eigen::Matrix3Xd points_w;
    points_w.resize(Eigen::NoChange, options_.n_random_landmarks_);
    for (int li = 0; li < options_.n_random_landmarks_; li++)
    {
      points_w.col(li) = points_w_vec[static_cast<size_t>(pt_start + li)];
    }

    approx_info_map.setOccupancyWorldPoints(points_w);
    approx_info_map.recomputeFactorLayer();
    for (const InfoMetricType& t : kValidMetric)
    {
      double v = 0.0;
      approx_info_map.getInfoMetricAt(Twc, t, &v);
      type_sum[t] += v;
    }
  }
  for (const InfoMetricType& t : kValidMetric)
  {
    thresh_map->insert({ t, type_sum[t] / options_.average_over_n_ });
    func_map->insert(
        { t, InfoPotentialFunc((*thresh_map)[t], options_.val_at_zero_) });
  }
}

template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const InformationPotential<T>& info_pot)
{
  os << "Info. Potentials: thresholds for different info. metrics:\n";
  for (const act_map::InfoMetricType& info_t :
       act_map::InformationPotential<T>::kValidMetric)
  {
    double info_val = info_pot.getMetricThresh(info_t);
    os << "- exact " << act_map::kInfoMetricNames.at(info_t) << ": " << info_val
       << std::endl;
    info_val = info_pot.getMetricThreshApproxVis(info_t);
    os << "- current approx. " << act_map::kInfoMetricNames.at(info_t) << ": "
       << info_val << std::endl;
  }

  return os;
}

}  // namespace act_map
