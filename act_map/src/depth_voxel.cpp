#include "act_map/depth_voxel.h"

#include <cmath>

#include "act_map/common.h"

namespace act_map
{
constexpr DepthVoxel::FloatType DepthVoxel::kInfD;
constexpr int DepthVoxel::kOpenCVType;
constexpr size_t DepthVoxel::kMinDIdx;
constexpr size_t DepthVoxel::kMaxDIdx;

DepthVoxel::~DepthVoxel()
{
}

void DepthVoxel::init(const rpg_common::Position& pos, const double deg_step)
{
  pos_ = pos.cast<FloatType>();
  step_deg_ = static_cast<FloatType>(deg_step);

  const int n_latitude = latitudeDegToIdx(90.0) + 1;
  const int n_longtitude = longtitudeDegToIdx(180.0) + 1;

  ray_depth_map_ = Eigen::Matrix<FloatType, -1, -1>::Constant(
      n_latitude, n_longtitude, kInfD);

  initialized_ = true;
}

std::ostream& operator<<(std::ostream& os, const DepthVoxel& ds)
{
  os << "Depth Sphere:\n";
  os << "- initialized: " << ds.initialized_ << std::endl;
  os << "- dimension: lat.:" << ds.ray_depth_map_.rows()
     << ", long.:" << ds.ray_depth_map_.cols() << std::endl;
  return os;
}

size_t DepthVoxel::numValidDepths() const
{
  size_t cnt = 0;
  for (int ri = 0; ri < ray_depth_map_.rows(); ri++)
  {
    for (int ci = 0; ci < ray_depth_map_.cols(); ci++)
    {
      if (ray_depth_map_(ri, ci) < kInfD)
      {
        cnt++;
      }
    }
  }
  return cnt;
}

size_t DepthVoxel::numMissingDepths() const
{
  size_t cnt = 0;
  for (int ri = 0; ri < ray_depth_map_.rows(); ri++)
  {
    for (int ci = 0; ci < ray_depth_map_.cols(); ci++)
    {
      if (ray_depth_map_(ri, ci) == kInfD)
      {
        cnt++;
      }
    }
  }
  return cnt;
}

void DepthVoxel::printMinMaxDepthMap() const
{
  std::cout << "Depthmap:\n"
            << " - min: " << ray_depth_map_.minCoeff() << std::endl
            << " - max: " << ray_depth_map_.maxCoeff() << std::endl;
}

void DepthVoxel::generateSampleTws(const rpg_common::Pose& Twc,
                                   const bool inc_top_down,
                                   rpg_common::PoseVec* Tcs)
{
  CHECK(Tcs);
  Tcs->clear();

  rpg_common::Pose Tc_fr = rpg_common::Pose();
  Tcs->push_back(Twc * Tc_fr);

  // right, back and left
  Eigen::Vector3d yaxis(0.0, 1.0, 0.0);
  std::vector<double> y_rot{ 0.5 * M_PI, 1.0 * M_PI, 1.5 * M_PI };

  rpg_common::Aligned<std::vector, Eigen::Vector3d> axies_vec{ yaxis };
  std::vector<std::vector<double>> ang_vec{ y_rot };
  if (inc_top_down)
  {
    // top and bottom
    Eigen::Vector3d xaxis(1.0, 0.0, 0.0);
    std::vector<double> x_rot{ +0.5 * M_PI, -0.5 * M_PI };
    axies_vec.push_back(xaxis);
    ang_vec.push_back(x_rot);
  }

  for (size_t idx = 0; idx < axies_vec.size(); idx++)
  {
    const Eigen::Vector3d& axis = axies_vec[idx];
    for (const double ang : ang_vec[idx])
    {
      rpg_common::Pose Tc_si;
      Tc_si.setIdentity();
      Tc_si.getRotation() = rpg_common::Rotation(Eigen::Vector3d(axis * ang));
      Tcs->push_back(Twc * Tc_si);
    }
  }
}

void DepthVoxel::setFromDepthImages(const vi_utils::PinholeCamVec& cam_vec,
                                    const std::vector<cv::Mat>& zd_imgs,
                                    const rpg_common::PoseVec& pose_vec,
                                    const bool z_depth)
{
  CHECK_EQ(cam_vec.size(), zd_imgs.size());
  CHECK_EQ(cam_vec.size(), pose_vec.size());
  for (size_t i = 0; i < cam_vec.size(); i++)
  {
    CHECK_EQ(cam_vec.at(i)->wAs<int>(), zd_imgs.at(i).cols) << i
                                                            << "th camera and "
                                                               "image are not "
                                                               "consistent "
                                                               "(check render "
                                                               "setting?).";
    CHECK_EQ(cam_vec.at(i)->hAs<int>(), zd_imgs.at(i).cols) << i
                                                            << "th camera and "
                                                               "image are not "
                                                               "consistent "
                                                               "(check render "
                                                               "setting?).";
  }
  const size_t num_d = zd_imgs.size();

  for (size_t i = 0; i < num_d; i++)
  {
    const vi_utils::PinholeCamPtr& cam = cam_vec[i];
    const cv::Mat& dimg = zd_imgs[i];
    CHECK_EQ(dimg.type(), kOpenCVType);
    const rpg_common::Pose& Twc = pose_vec[i];

    cam->computeBearingVectors();

    for (int ux = 0; ux < cam->wAs<int>(); ux++)
    {
      for (int uy = 0; uy < cam->hAs<int>(); uy++)
      {
        Eigen::Vector3d fc;
        cam->getBearingAtPixel(ux, uy, &fc);
        CHECK_DOUBLE_EQ(fc(2), 1.0);
        Eigen::Vector3d vec_w;
        if (z_depth)
        {
          FloatType zd = static_cast<FloatType>(dimg.at<float>(uy, ux));
          vec_w = Twc.getRotation().rotate(fc * zd);
        }
        else
        {
          vec_w = Twc.getRotation().rotate(
              fc.normalized() * static_cast<FloatType>(dimg.at<float>(uy, ux)));
        }
        setDepthFromWorldVector(vec_w.cast<FloatType>());
      }
    }
  }
}

void DepthVoxel::queryDepthImage(const vi_utils::PinholeCamVec& cam_vec,
                                 const rpg_common::PoseVec& pose_vec,
                                 std::vector<cv::Mat>* dimgs) const
{
  CHECK_EQ(cam_vec.size(), pose_vec.size());
  CHECK(dimgs);
  dimgs->resize(pose_vec.size());
  const size_t num_d = cam_vec.size();

  for (size_t i = 0; i < num_d; i++)
  {
    const vi_utils::PinholeCamPtr& cam = cam_vec[i];
    dimgs->at(i).create(static_cast<int>(cam->h()), static_cast<int>(cam->w()),
                        kOpenCVType);
    const rpg_common::Pose& Twc = pose_vec[i];

    cam->computeBearingVectors();

    for (int ux = 0; ux < cam->w(); ux++)
    {
      for (int uy = 0; uy < cam->h(); uy++)
      {
        Eigen::Vector3d fc;
        cam->getBearingAtPixel(ux, uy, &fc);
        Eigen::Vector3d vec_w = Twc.getRotation().rotate(fc);
        FloatType zd = queryDepthWorldVector(vec_w.cast<FloatType>());
        dimgs->at(i).at<float>(uy, ux) = zd;
      }
    }
  }
}

void DepthVoxel::getVisibleIdxFromPoints(const rpg::PositionVec& points,
                                         VisIdx* vis_idx,
                                         const MinMaxD& min_max_d) const
{
  Vec3Vec vec_w_s;
  vec_w_s.reserve(points.size());
  for (size_t pt_i = 0; pt_i < points.size(); pt_i++)
  {
    vec_w_s.emplace_back(points[pt_i].cast<FloatType>() - pos_);
  }

  getVisibleIdxInternal(vec_w_s, vis_idx, min_max_d);
}

void DepthVoxel::getVisibleIdxInternal(const Vec3Vec& vec_w_s, VisIdx* vis_idx,
                                       const MinMaxD& min_max_d) const
{
  CHECK(vis_idx);
  vis_idx->clear();

  size_t viz_cnt = 0;
  // 1st pass: according to depth map
  std::vector<bool> is_visible;
  for (size_t vec_i = 0; vec_i < vec_w_s.size(); vec_i++)
  {
    const Vec3& vec_w = vec_w_s[vec_i];
    FloatType d = queryDepthWorldVector(vec_w, true);
    const FloatType cur_d = vec_w.norm();
    if ((cur_d > d + static_cast<FloatType>(0.01)) ||
        cur_d < min_max_d[kMinDIdx] || cur_d > min_max_d[kMaxDIdx])
    {
      is_visible.push_back(false);
    }
    else
    {
      is_visible.push_back(true);
      viz_cnt++;
    }
  }
  VLOG(10) << "Visibility check: " << viz_cnt
           << " visible points according to depth map.";

  // 2nd pass: multiple points fall in the same grid
  DepthMat depth_buffer;
  Eigen::Matrix<int, -1, -1> idx_mat;
  idx_mat.resize(ray_depth_map_.rows(), ray_depth_map_.cols());
  idx_mat.setConstant(-1);
  depth_buffer.resize(ray_depth_map_.rows(), ray_depth_map_.cols());
  depth_buffer.setConstant(kInfD);
  viz_cnt = 0;
  for (size_t vec_i = 0; vec_i < vec_w_s.size(); vec_i++)
  {
    if (!is_visible[vec_i])
    {
      continue;
    }
    const Vec3& vec_w = vec_w_s[vec_i];
    int ridx, cidx;
    std::tie(ridx, cidx) = vecWorldToDepthMapIndices(vec_w);

    if (vec_w.norm() < depth_buffer(ridx, cidx))
    {
      depth_buffer(ridx, cidx) = vec_w.norm();
      idx_mat(ridx, cidx) = static_cast<int>(vec_i);
    }
  }

  for (int ridx = 0; ridx < idx_mat.rows(); ridx++)
  {
    for (int cidx = 0; cidx < idx_mat.cols(); cidx++)
    {
      int idx = idx_mat(ridx, cidx);
      if (idx >= 0)
      {
        vis_idx->insert(static_cast<size_t>(idx));
      }
    }
  }

  VLOG(10) << "Visibility check: " << vis_idx->size()
           << " visible points after point occlusion.";
}

void DepthVoxel::getDepthPoints(Vec3dVec* points_w) const
{
  CHECK(points_w);
  points_w->clear();
  for (int ridx = 0; ridx < ray_depth_map_.rows(); ridx++)
  {
    for (int cidx = 0; cidx < ray_depth_map_.cols(); cidx++)
    {
      FloatType rd = ray_depth_map_(ridx, cidx);
      if (rd == kInfD)
      {
        continue;
      }
      Vec3 f;
      depthMapIndicesToVecWorld(ridx, cidx, &f);
      f.normalize();
      f *= rd;

      points_w->emplace_back((f + pos_).cast<double>());
    }
  }
}

bool DepthVoxel::isTheSame(const DepthVoxel& v) const
{
  bool is_the_same = true;
  constexpr FloatType kTolerance = 1e-8f;
  is_the_same &= (pos_.isApprox(v.pos_));
  is_the_same &= (std::fabs(step_deg_ - v.step_deg_) < 1e-8);
  is_the_same &=
      (ray_depth_map_ - v.rayDepthMatCRef()).cwiseAbs().maxCoeff() < kTolerance;
  return is_the_same;
}

}  // namespace act_map
