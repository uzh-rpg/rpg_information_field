#pragma once

#include <iostream>
#include <memory>

#include <rpg_common/pose.h>
#include <rpg_common/fs.h>
#include <rpg_common/timer.h>

#include "act_map/voxblox/core/common.h"
#include "act_map/voxblox/core/layer.h"
#include "act_map/voxblox/core/voxel.h"
#include "act_map/voxblox/io/layer_io.h"
#include "act_map/voxblox/integrator/occupancy_integrator.h"
#include "act_map/quadratic_vis_score.h"
#include "act_map/common.h"
#include "act_map/sampler.h"
#include "act_map/positional_factor_layer_integrator.h"
#include "act_map/collision_check.h"
#include "act_map/pos_factor_layer_evaluator.h"
#include "act_map/depth_map.h"
#include "act_map/visibility_checker.h"
#include "act_map/info_calculator.h"
#include "act_map/optim_orient.h"

namespace act_map
{
struct ActMapOptions
{
  LayerOptions occ_layer_options_;
  voxblox::OccupancyIntegrator::Config occ_integrator_options_;

  std::vector<QuadVisScoreOptions> vis_options_;

  LayerOptions pos_factor_layer_options_;
  act_map::PositionalFactorLayerIntegratorOptions pos_fac_integrator_options_;

  VisibilityCheckerOptions vis_checker_options_;

  bool use_collision_checker_ = false;
  utils::CollisionCheckerOptions col_ops_;

  EvaluateStrategy eval_method = EvaluateStrategy::kInterpolation;
};

template <typename T>
class ActMap
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ActMap() = delete;
  ActMap(const ActMap&) = delete;
  ActMap& operator=(const ActMap&) = delete;

  ActMap(const ActMapOptions& options);
  ~ActMap()
  {
  }

  template <typename KVType>
  friend std::ostream& operator<<(std::ostream& os, const ActMap<KVType>& am);

  ActMapOptions options_;

  inline voxblox::Layer<voxblox::OccupancyVoxel>::Ptr occLayerPtr()
  {
    return occ_layer_;
  }

  inline const voxblox::Layer<voxblox::OccupancyVoxel>& occLayerCRef() const
  {
    return *occ_layer_;
  }

  // positional factor layer related
  void allocateFactorLayerUniform(const std::vector<double>& ranges);

  inline typename voxblox::Layer<T>::Ptr kerLayerPtr()
  {
    return pos_factor_layer_;
  }

  inline const voxblox::Layer<T>& kerLayerCRef() const
  {
    return *pos_factor_layer_;
  }

  inline VisibilityChecker& visCheckerRef()
  {
    return *vis_checker_;
  }

  inline const VisibilityChecker& visCheckerCRef() const
  {
    return *vis_checker_;
  }

  inline void setOccupancyLayerValid()
  {
    act_map::utils::setLayerValid(occ_layer_.get());
  }

  inline void setFactorLayerValid()
  {
    act_map::utils::setLayerValid(pos_factor_layer_.get());
  }

  // map update
  void integratePointCloudOccupancy(const rpg::Pose& T_w_c,
                                    const Eigen::Matrix3Xd& points_c);

  void setOccupancyWorldPoints(
      const Eigen::Matrix3Xd& points_w,
      const Eigen::Matrix3Xd& aver_views_from_pt = Eigen::Matrix3Xd());

  void recomputeFactorLayer();

  void updateFactorLayerIncremental();

  void addRegionToFactorLayer(const rpg::Pose& Twb,
                              const std::vector<double>& ranges);

  // map query
  bool getInfoMetricAt(
      const rpg::Pose& Twc, const InfoMetricType& info_t, double* val,
      typename PosFactorLayerEvaluator<T>::GradT* dpos = nullptr,
      typename PosFactorLayerEvaluator<T>::GradT* drot_g = nullptr) const;

  bool getInfoMetricFromPC(
      const rpg::Pose& Twc, const InfoMetricType& info_t, double* val,
      typename PosFactorLayerEvaluator<T>::GradT* dpos = nullptr,
      typename PosFactorLayerEvaluator<T>::GradT* drot_g = nullptr) const;

  // best views
  void getBestViewsAt(const size_t cam_id, const int samples_per_side,
                      const bool use_sampling, const bool only_updated,
                      Vec3dVec* vox_cs, Vec3dVec* best_views,
                      std::vector<double>* values,
                      voxblox::LongIndexVector* global_idxs) const;

  void getBestViewFromNearestVoxel(const Eigen::Vector3d& pos,
                                   const InfoMetricType& type,
                                   Eigen::Vector3d* vox_cs,
                                   Eigen::Vector3d* bview) const;

  // this should only be used in simulation: only consider the visiblity
  // according to cameras
  void getBestViewFromPCNoVisCheck(const Eigen::Vector3d& pos,
                                   const InfoMetricType& type,
                                   Eigen::Vector3d* bview) const;

  // alternative ways of calculate FIM
  void getFIMFromPC(const rpg::Pose& Twc, Info* fim) const;
  void getFIMFromNearestVoxel(const rpg::Pose& Twc, Eigen::Vector3d* vox_cs,
                              Info* fim) const;

  // query map properties
  inline InfoMetricTypeVec supportedInfoMetricTypes() const
  {
    return supportedInfoMetricTypesVoxT<T>();
  }
  inline bool supportFullFIM() const
  {
    return supportFullFIMVoxT<T>();
  }

  // misc accessors
  inline const voxblox::BlockIndexList&
  getAccumulatedUpdatedFactorBlocksIndices() const
  {
    return accumulated_updated_kblk_idxs_;
  }
  inline void clearAccumulatedUpdatedFactorBlockIndices()
  {
    accumulated_updated_kblk_idxs_.clear();
  }
  inline double updatedBlkRatioOcc() const
  {
    return utils::getUpdatedRatio<OccupancyVoxel>(*occ_layer_);
  }
  inline double updatedBlkRatioFactorLayer() const
  {
    return utils::getUpdatedRatio<T>(*pos_factor_layer_);
  }
  inline double accumulatedUpdatedBlkRatioFactorLayer() const
  {
    size_t n_all_blks = pos_factor_layer_->getNumberOfAllocatedBlocks();
    if (n_all_blks == 0)
    {
      return 0.0;
    }
    return accumulated_updated_kblk_idxs_.size() / n_all_blks;
  }

  inline const voxblox::HierarchicalIndexMap& getLastDeletedOccPoints()
  {
    return last_deleted_occ_pts_;
  }

  inline const voxblox::HierarchicalIndexMap& getLastAddedOccPoints()
  {
    return last_added_occ_pts_;
  }

  inline const voxblox::IndexSet getNewlyAllocatedFactorBlockIndices()
  {
    return kblk_idxs_to_recompute_;
  }

  inline const voxblox::IndexSet getPrevAllocatedFactorBlockIndices()
  {
    return kblk_idxs_to_update_;
  }

  inline double occBlockHalfDiagonal() const
  {
    return occ_block_half_diagonal_;
  }

  inline double posFactorBlockHalfDiagonal() const
  {
    return pos_fac_block_half_diagonal_;
  }

  inline size_t posFactorVoxelNumCoefs() const
  {
    T vox;
    return vox.numCoefs();
  }

  void getViewDirsOfOccupiedVoxels(act_map::Vec3dVec* view_dirs) const;
  void getCentersOfOccupiedVoxels(act_map::Vec3dVec* points_w) const;
  void getCentersOfOccupiedVoxels(act_map::Vec3dVec* blk_cs,
                                  act_map::V3dVecVec* blk_points_w,
                                  act_map::V3dVecVec* blk_pts_view_dirs) const;
  inline size_t numOccupiedVoxels() const
  {
    return utils::countNumOccupiedVoxels(
        *occ_layer_, options_.occ_integrator_options_.threshold_occupancy);
  }

  void activateBlocksByDistance(const Eigen::Vector3d& pos,
                                const double thresh);

  void saveLayers(const std::string& occ_layer_fn,
                  const std::string& pos_factor_layer_fn) const;

  void loadLayers(const std::string& occ_layer_fn,
                  const std::string& pos_factor_layer_fn);

  void resetIncUpdateVars();

  void clearMap();

  void prepareInfoFromPointCloud();
  void cachePointsAndViewDirs(const bool force_update) const;

  inline const act_map::Vec3dVec& cachedPoints() const
  {
    return pc_positions_;
  }

  inline const act_map::Vec3dVec& cachedViewDirs() const
  {
    return pc_views_;
  }

  inline void sampleRotations() const
  {
    if (rot_samples_.empty())
    {
      utils::sampleRotation(10.0, &rot_samples_);
    }
  }

private:
  // occupancy layer
  mutable std::mutex occ_mutex_;
  OccupancyLayer::Ptr occ_layer_;
  std::unique_ptr<voxblox::OccupancyIntegrator> occ_integrator_;

  // visibility checker: used in integrator as well as baseline
  VisibilityCheckerPtr vis_checker_ = nullptr;

  // positional factor layer
  mutable std::mutex pos_factor_mutex_;
  typename voxblox::Layer<T>::Ptr pos_factor_layer_;
  std::unique_ptr<act_map::PositionalFactorLayerIntegrator<T>>
      pos_fac_integrator_;

  std::vector<QuadraticVisScore> vis_scores_;  // only used for visualization

  // evaluator
  typename act_map::PosFactorLayerEvaluator<T>::Ptr evaluator_;

  // update results
  // We do not lock this one: shoule be used after the update is done
  voxblox::BlockIndexList accumulated_updated_kblk_idxs_;
  // these varibles will be used by both occupancy integration and factor
  // integration, should be locked
  voxblox::HierarchicalIndexMap last_deleted_occ_pts_;
  voxblox::HierarchicalIndexMap last_added_occ_pts_;

  // used to maintain the positional factor layer blocks
  // These variable are used both by the expansion and update.
  voxblox::IndexSet kblk_idxs_to_update_;
  voxblox::IndexSet kblk_idxs_to_recompute_;

  // other variables
  double occ_block_half_diagonal_;
  double pos_fac_block_half_diagonal_;

  // for baseline evaluation
  mutable act_map::Vec3dVec pc_positions_;
  mutable Eigen::Matrix3Xd pc_pos_mat_;
  mutable act_map::Vec3dVec pc_views_;
  act_map::InfoCalculatorPtr info_cal_ptr_ = nullptr;

  // sampled rotaiton for optimal vidw calculation
  mutable rpg::RotationVec rot_samples_;
};

template <typename T>
using ActMapPtr = std::shared_ptr<ActMap<T>>;
using TraceMap = ActMap<QuadTraceVoxel>;
using TraceMapPtr = std::shared_ptr<TraceMap>;
using InfoMap = ActMap<QuadInfoVoxel>;
using InfoMapPtr = std::shared_ptr<InfoMap>;
using GPInfoMap = ActMap<GPInfoVoxel>;
using GPInfoMapPtr = std::shared_ptr<GPInfoMap>;
using GPTraceMap = ActMap<GPTraceVoxel>;
using GPTraceMapPtr = std::shared_ptr<GPTraceMap>;

template <typename T>
ActMap<T>::ActMap(const ActMapOptions& options) : options_(options)
{
  occ_layer_ = std::make_shared<voxblox::Layer<voxblox::OccupancyVoxel>>(
      options_.occ_layer_options_.vox_size,
      options_.occ_layer_options_.vox_per_side);
  occ_integrator_.reset(new voxblox::OccupancyIntegrator(
      options_.occ_integrator_options_, occ_layer_.get()));

  vis_checker_ =
      VisibilityChecker::createVisibilityChecker(options_.vis_checker_options_);

  pos_factor_layer_ = std::make_shared<voxblox::Layer<T>>(
      options_.pos_factor_layer_options_.vox_size,
      options_.pos_factor_layer_options_.vox_per_side);
  pos_fac_integrator_.reset(new PositionalFactorLayerIntegrator<T>(
      options_.pos_fac_integrator_options_, occ_layer_, pos_factor_layer_,
      vis_checker_));

  CHECK_GT(options_.vis_options_.size(), 0u);
  CHECK_EQ(options_.vis_options_.size(), 1u) << "Currently only one caemra is "
                                                "supported.";
  for (size_t i = 0; i < options_.vis_options_.size(); i++)
  {
    const QuadVisScoreOptions& op = options_.vis_options_[i];
    vis_scores_.emplace_back(op.half_fov_rad);
    vis_scores_[i].initSecondOrderApprox(op.boundary_to_mid_ratio,
                                         op.boundary_value);
  }

  evaluator_ = std::make_shared<act_map::PosFactorLayerEvaluator<T>>(
      pos_factor_layer_.get());
  const QuadraticVisScore& vis0 = vis_scores_[0];
  evaluator_->setQuadraticCoefs(vis0.k1(), vis0.k2(), vis0.k3());

  occ_block_half_diagonal_ = occ_layer_->block_size() * 0.866;
  pos_fac_block_half_diagonal_ = pos_factor_layer_->block_size() * 0.866;

  resetIncUpdateVars();
}

template <typename T>
void ActMap<T>::resetIncUpdateVars()
{
  accumulated_updated_kblk_idxs_.clear();
  last_deleted_occ_pts_.clear();
  last_added_occ_pts_.clear();
  kblk_idxs_to_update_.clear();
  kblk_idxs_to_recompute_.clear();
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const ActMap<T>& am)
{
  os << "ActMap:\n"
     << "- Occupancy:\n"
     << "  - voxel size: " << am.occ_layer_->voxel_size() << std::endl
     << "  - voxel per side: " << am.occ_layer_->voxels_per_side() << std::endl
     << "  - block size: " << am.occ_layer_->block_size() << std::endl;
  return os;
}

template <typename T>
void ActMap<T>::getViewDirsOfOccupiedVoxels(act_map::Vec3dVec* view_dirs) const
{
  std::lock_guard<std::mutex> lock_occ(occ_mutex_);
  utils::getViewDirsOfOccupiedVoxels(
      *occ_layer_, options_.pos_fac_integrator_options_.occ_thresh_, view_dirs);
}

template <typename T>
void ActMap<T>::getCentersOfOccupiedVoxels(act_map::Vec3dVec* points_w) const
{
  std::lock_guard<std::mutex> lock_occ(occ_mutex_);
  utils::getCentersOfOccupiedVoxels(
      *occ_layer_, options_.pos_fac_integrator_options_.occ_thresh_, points_w);
}

template <typename T>
void ActMap<T>::getCentersOfOccupiedVoxels(
    act_map::Vec3dVec* blk_cs, act_map::V3dVecVec* blk_points_w,
    act_map::V3dVecVec* blk_pts_view_dirs) const
{
  std::lock_guard<std::mutex> lock_occ(occ_mutex_);
  CHECK_NOTNULL(blk_cs);
  utils::getCentersOfOccupiedVoxels(
      *occ_layer_, options_.pos_fac_integrator_options_.occ_thresh_, blk_cs,
      blk_points_w, blk_pts_view_dirs);
}

template <typename T>
void ActMap<T>::integratePointCloudOccupancy(const rpg::Pose& T_w_c,
                                             const Eigen::Matrix3Xd& points_c)
{
  voxblox::Pointcloud pc_c;
  act_map::eigenKXToVecKVec(points_c, &pc_c);
  {
    std::lock_guard<std::mutex> lock(occ_mutex_);
    occ_integrator_->integratePointCloud(T_w_c, pc_c, &last_deleted_occ_pts_,
                                         &last_added_occ_pts_);
  }
}

template <typename T>
void ActMap<T>::setOccupancyWorldPoints(
    const Eigen::Matrix3Xd& points_w,
    const Eigen::Matrix3Xd& aver_views_from_pt)
{
  bool set_view = false;
  if (aver_views_from_pt.cols() > 0)
  {
    CHECK_EQ(aver_views_from_pt.cols(), points_w.cols());
    set_view = true;
    VLOG(1) << "Will set average view directions in the occupancy layer.";
  }
  Vec3dVec points_vec;
  eigenKXToVecKVec(points_w, &points_vec);
  utils::allocateBlocksByCoordinatesBatch(points_vec, occ_layer_.get(), nullptr,
                                          nullptr);
  for (size_t idx = 0; idx < points_vec.size(); idx++)
  {
    const Eigen::Vector3d& pt = points_vec[idx];
    voxblox::OccupancyVoxel* vox = occ_layer_->getVoxelPtrByCoordinates(pt);
    CHECK(vox);
    setOccupancyVoxelOccupied(vox);
    if (set_view)
    {
      vox->aver_view_from_pt =
          aver_views_from_pt.col(static_cast<int>(idx)).cast<float>();
    }
  }
}

template <typename T>
void ActMap<T>::allocateFactorLayerUniform(const std::vector<double>& ranges)
{
  rpg::PositionVec points;
  utils::generateUniformPointsWithin(
      options_.pos_factor_layer_options_.vox_size, ranges, &points);

  {
    std::lock_guard<std::mutex> lock_ker(pos_factor_mutex_);
    std::lock_guard<std::mutex> lock_occ(occ_mutex_);
    voxblox::BlockIndexList new_blks;
    voxblox::IndexSet covered_blks;
    CHECK(pos_factor_layer_);
    utils::allocateBlocksByCoordinatesBatch(points, pos_factor_layer_.get(),
                                            &new_blks, &covered_blks);
    int n_masked = 0;
    if (options_.use_collision_checker_)
    {
      for (const voxblox::BlockIndex& idx : new_blks)
      {
        typename voxblox::Block<T>::Ptr blk_ptr =
            pos_factor_layer_->getBlockPtrByIndex(idx);
        n_masked += utils::maskCollidedVoxels(
            *occ_layer_, blk_ptr.get(),
            options_.pos_fac_integrator_options_.occ_thresh_,
            options_.col_ops_);
      }
    }
    else
    {
      VLOG(3) << "Not using collision checker.";
    }
    VLOG(3) << "Factor Layer Expand: allocated " << new_blks.size()
            << " new blocks, "
            << "covered " << covered_blks.size() << " blocks, "
            << "and masked " << n_masked << " voxels in covered blocks.";
    kblk_idxs_to_recompute_.insert(new_blks.begin(), new_blks.end());
  }
}

template <typename T>
void ActMap<T>::recomputeFactorLayer()
{
  Vec3dVec points_w;
  Vec3dVec view_dirs;
  resetIncUpdateVars();
  getCentersOfOccupiedVoxels(&points_w);
  getViewDirsOfOccupiedVoxels(&view_dirs);
  {
    std::lock_guard<std::mutex> lock_ker(pos_factor_mutex_);
    voxblox::BlockIndexList cand_blks;
    pos_factor_layer_->getAllAllocatedBlocks(&cand_blks);
    VLOG(5) << "Recompute factor layer: to process " << points_w.size()
            << " points.";
    VLOG(5) << "Recompute factor layer: to process " << cand_blks.size()
            << " blocks.";
    pos_fac_integrator_->recomputeFactorLayer(points_w, view_dirs, cand_blks,
                                              &accumulated_updated_kblk_idxs_);
    VLOG(5) << "Recompute factor layer: upated "
            << accumulated_updated_kblk_idxs_.size() << " blocks.";
  }
}

template <typename T>
void ActMap<T>::updateFactorLayerIncremental()
{
  std::lock_guard<std::mutex> lock_ker(pos_factor_mutex_);

  voxblox::BlockIndexList del_block_idxs;
  voxblox::BlockIndexList add_block_idxs;
  rpg::Timer timer;
  timer.start();
  pos_fac_integrator_->deletePointsFromFactorLayer(
      last_deleted_occ_pts_, kblk_idxs_to_update_, &del_block_idxs);

  pos_fac_integrator_->addPointsToFactorLayer(
      last_added_occ_pts_, kblk_idxs_to_update_, &add_block_idxs);

  VLOG(3) << "Factor Layer Updater: "
          << "processed " << last_deleted_occ_pts_.size() << " deleted points, "
          << last_added_occ_pts_.size() << " added points.";
  VLOG(2) << "Update factor took " << timer.stop() * 1000 << " ms.";

  timer.start();
  voxblox::BlockIndexList recompute_blk_idxs;
  if (kblk_idxs_to_recompute_.size() != 0)
  {
    Vec3dVec blk_cs;
    V3dVecVec blk_points;
    V3dVecVec blk_pts_view_dirs;
    getCentersOfOccupiedVoxels(&blk_cs, &blk_points, &blk_pts_view_dirs);
    pos_fac_integrator_->recomputeFactorLayer(
        blk_cs, blk_points, blk_pts_view_dirs, kblk_idxs_to_recompute_,
        &recompute_blk_idxs);
  }
  VLOG(2) << "Recomptue factors took " << timer.stop() * 1000 << " ms.";

  const size_t n_to_recompute = kblk_idxs_to_recompute_.size();
  const size_t n_recomputed = recompute_blk_idxs.size();

  voxblox::IndexSet merged_idxs(del_block_idxs.begin(), del_block_idxs.end());
  merged_idxs.insert(add_block_idxs.begin(), add_block_idxs.end());
  const size_t n_modified = merged_idxs.size();
  merged_idxs.insert(recompute_blk_idxs.begin(), recompute_blk_idxs.end());
  const size_t n_updated = merged_idxs.size();
  accumulated_updated_kblk_idxs_.insert(accumulated_updated_kblk_idxs_.end(),
                                        merged_idxs.begin(), merged_idxs.end());
  const size_t n_accumulate_updated = accumulated_updated_kblk_idxs_.size();
  VLOG(3) << "Factor Layer Updater: "
          << "to recompute " << n_to_recompute << " blocks, "
          << "recomputed " << n_recomputed << " blocks, "
          << "modified " << n_modified << " blocks, "
          << "total updated " << n_updated << " blocks, "
          << "accumulated updated " << n_accumulate_updated << " blocks.";

  kblk_idxs_to_update_.insert(kblk_idxs_to_recompute_.begin(),
                              kblk_idxs_to_recompute_.end());
  kblk_idxs_to_recompute_.clear();
}

template <typename T>
bool ActMap<T>::getInfoMetricAt(
    const rpg::Pose& Twc, const InfoMetricType& info_t, double* val,
    typename PosFactorLayerEvaluator<T>::GradT* dpos,
    typename PosFactorLayerEvaluator<T>::GradT* drot_g) const
{
  bool res = false;
  if (options_.eval_method == EvaluateStrategy::kNearestNeighbour)
  {
    res = evaluator_->getValueNearest(Twc, info_t, val, dpos, drot_g);
  }
  else if (options_.eval_method == EvaluateStrategy::kInterpolation)
  {
    res = evaluator_->getValueInterpolation(Twc, info_t, val, dpos, drot_g);
  }
  else
  {
    LOG(FATAL) << "Unknown evaluation strategy.";
  }

  if (!res)
  {
    LOG(WARNING) << "Get info metric failed.";
    (*val) = 0.0;
    if (dpos)
    {
      dpos->setZero();
    }
    if (drot_g)
    {
      drot_g->setZero();
    }
  }

  return res;
}

template <typename T>
bool ActMap<T>::getInfoMetricFromPC(
    const rpg::Pose& Twc, const InfoMetricType& info_t, double* val,
    typename PosFactorLayerEvaluator<T>::GradT* dpos,
    typename PosFactorLayerEvaluator<T>::GradT* drot_g) const
{
  CHECK(info_cal_ptr_) << "need to initialize information calcualtor...";
  CHECK(pc_positions_.size() != 0) << "need to have the pointcloud cache "
                                      "first...";
  return info_cal_ptr_->calculateInfoMetricAt(Twc, pc_positions_, pc_views_,
                                              info_t, val, dpos, drot_g);
}

template <typename T>
void ActMap<T>::addRegionToFactorLayer(const rpg::Pose& Twb,
                                       const std::vector<double>& ranges)
{
  Eigen::Vector3d center = Twb.getPosition();
  std::vector<double> alloc_ranges(6);
  if (ranges.size() == 3)
  {
    std::vector<double> half_ranges;
    for (const double v : ranges)
    {
      CHECK_GT(v, 0.1);
      half_ranges.push_back(0.5 * v);
    }
    alloc_ranges = { center.x() - half_ranges[0], center.x() + half_ranges[0],
                     center.y() - half_ranges[1], center.y() + half_ranges[1],
                     center.z() - half_ranges[2], center.z() + half_ranges[2] };
  }
  else if (ranges.size() == 6)
  {
    alloc_ranges = { center.x() + ranges[0], center.x() + ranges[1],
                     center.y() + ranges[2], center.y() + ranges[3],
                     center.z() + ranges[4], center.z() + ranges[5] };
  }
  allocateFactorLayerUniform(alloc_ranges);
}

template <typename T>
void ActMap<T>::getBestViewsAt(const size_t cam_id, const int samples_per_side,
                               const bool use_sampling, const bool only_updated,
                               Vec3dVec* vox_cs, Vec3dVec* best_views,
                               std::vector<double>* values,
                               voxblox::LongIndexVector* global_idxs) const
{
  LOG(WARNING) << "Default implementation: best view visualization not "
                  "supported for this factor,"
                  " doing nothing.";
}

template <typename T>
void ActMap<T>::getBestViewFromNearestVoxel(const Eigen::Vector3d& pos,
                                            const InfoMetricType& type,
                                            Eigen::Vector3d* vox_cs,
                                            Eigen::Vector3d* bview) const
{
  LOG(WARNING) << "Default implementation of get best view from voxel, doing "
                  "nothing.";
}

template <typename T>
void ActMap<T>::getBestViewFromPCNoVisCheck(const Eigen::Vector3d& pos,
                                            const InfoMetricType& type,
                                            Eigen::Vector3d* bview) const
{
  CHECK_NOTNULL(bview);
  CHECK(pc_positions_.size() != 0) << "need to have the pointcloud cache "
                                      "first...";
  CHECK(vis_checker_->camPtr()) << "need to have camera at the visibility "
                                   "checker...";
  sampleRotations();
  double optim_val;
  optim_orient::getOptimalViewFromExactInfo(rot_samples_, pos, pc_pos_mat_,
                                            *vis_checker_->camPtr(), type,
                                            bview, &optim_val);
}

template <typename T>
void ActMap<T>::getFIMFromPC(const rpg::Pose& Twc, Info* fim) const
{
  CHECK_NOTNULL(fim);
  CHECK(info_cal_ptr_) << "need to initialize information calcualtor...";
  CHECK(pc_positions_.size() != 0) << "need to have the pointcloud cache "
                                      "first...";
  info_cal_ptr_->calculateInfoAt(Twc, pc_positions_, pc_views_, fim);
}

template <typename T>
void ActMap<T>::getFIMFromNearestVoxel(const rpg::Pose& Twc,
                                       Eigen::Vector3d* vox_c, Info* fim) const
{
  CHECK(!this->supportFullFIM());
  LOG(WARNING) << "Current voxel type does not support full fim, doing nothing";
}

template <typename T>
void ActMap<T>::activateBlocksByDistance(const Eigen::Vector3d& pos,
                                         const double thresh)
{
  voxblox::BlockIndexList reactivated_idxs;
  voxblox::BlockIndexList deactivated_idxs;

  std::lock_guard<std::mutex> lock(pos_factor_mutex_);

  voxblox::BlockIndexList blk_idxs;
  pos_factor_layer_->getAllAllocatedBlocks(&blk_idxs);
  for (const voxblox::BlockIndex& idx : blk_idxs)
  {
    typename voxblox::Block<T>::Ptr blk_ptr =
        pos_factor_layer_->getBlockPtrByIndex(idx);
    Eigen::Vector3d c;
    getBlockCenterFromBlk(*blk_ptr, &c);

    if ((c - pos).norm() < thresh)
    {
      if (!blk_ptr->activated())
      {
        reactivated_idxs.push_back(idx);
        blk_ptr->set_activated(true);
      }
    }
    else
    {
      if (blk_ptr->activated())
      {
        deactivated_idxs.push_back(idx);
        blk_ptr->set_activated(false);
      }
    }
  }

  VLOG(3) << "Blocks: Active -> Deactive: " << deactivated_idxs.size()
          << ", Deactive -> Active: " << reactivated_idxs.size();
  for (const voxblox::BlockIndex& idx : deactivated_idxs)
  {
    kblk_idxs_to_recompute_.erase(idx);
    kblk_idxs_to_update_.erase(idx);
  }
  for (const voxblox::BlockIndex& idx : reactivated_idxs)
  {
    kblk_idxs_to_recompute_.insert(idx);
  }
}

template <typename T>
void ActMap<T>::saveLayers(const std::string& occ_layer_fn,
                           const std::string& pos_factor_layer_fn) const
{
  rpg::Timer timer;
  {
    std::lock_guard<std::mutex> ker_lock(pos_factor_mutex_);
    LOG(WARNING) << "Saving factor layer...";
    if (rpg::fs::fileExists(pos_factor_layer_fn))
    {
      LOG(WARNING) << pos_factor_layer_fn << " exists, going to overwrite...";
    }
    timer.start();
    voxblox::io::SaveLayer<T>(*pos_factor_layer_, pos_factor_layer_fn, true);
    VLOG(1) << "Saving factor layer took " << timer.stop() * 1000 << " ms";
  }
  {
    std::lock_guard<std::mutex> occ_lock(occ_mutex_);
    LOG(WARNING) << "Saving occupancy layer...";
    if (rpg::fs::fileExists(occ_layer_fn))
    {
      LOG(WARNING) << occ_layer_fn << " exists, going to overwrite...";
    }
    timer.start();
    voxblox::io::SaveLayer<OccupancyVoxel>(*occ_layer_, occ_layer_fn, true);
    VLOG(1) << "Saving occupancy layer took " << timer.stop() * 1000 << " ms";
  }
}

template <typename T>
void ActMap<T>::loadLayers(const std::string& occ_layer_fn,
                           const std::string& pos_factor_layer_fn)
{
  bool loaded = false;
  if (!occ_layer_fn.empty())
  {
    std::lock_guard<std::mutex> occ_lock(occ_mutex_);
    CHECK(rpg::fs::fileExists(occ_layer_fn)) << occ_layer_fn;
    bool suc = voxblox::io::LoadBlocksFromFile(
        occ_layer_fn,
        voxblox::Layer<voxblox::OccupancyVoxel>::BlockMergingStrategy::kReplace,
        false, occ_layer_.get());
    if (!suc)
    {
      LOG(WARNING) << "Failed to load occupancy layer.";
    }
    else
    {
      CHECK_EQ(options_.occ_layer_options_.vox_size, occ_layer_->voxel_size());
      CHECK_EQ(options_.occ_layer_options_.vox_per_side,
               occ_layer_->voxels_per_side());
      setOccupancyLayerValid();
      VLOG(1) << "Loaded " << occ_layer_->getNumberOfAllocatedBlocks()
              << " occupancy blocks.";
    }
    loaded = suc;
  }
  if (!pos_factor_layer_fn.empty())
  {
    std::lock_guard<std::mutex> ker_lock(pos_factor_mutex_);
    CHECK(rpg::fs::fileExists(pos_factor_layer_fn)) << pos_factor_layer_fn;
    bool suc = voxblox::io::LoadBlocksFromFile(
        pos_factor_layer_fn, voxblox::Layer<T>::BlockMergingStrategy::kReplace,
        false, pos_factor_layer_.get());
    if (!suc)
    {
      LOG(WARNING) << "Failed to load factor layer.";
    }
    else
    {
      CHECK_EQ(options_.pos_factor_layer_options_.vox_size,
               pos_factor_layer_->voxel_size());
      CHECK_EQ(options_.pos_factor_layer_options_.vox_per_side,
               pos_factor_layer_->voxels_per_side());
      setFactorLayerValid();
      VLOG(1) << "Loaded " << pos_factor_layer_->getNumberOfAllocatedBlocks()
              << " factor blocks.";
    }
    loaded = true;
  }
  if (loaded)
  {
    resetIncUpdateVars();
  }
}

template <typename T>
void ActMap<T>::clearMap()
{
  std::lock_guard<std::mutex> occ_lock(occ_mutex_);
  std::lock_guard<std::mutex> ker_lock(pos_factor_mutex_);
  pos_factor_layer_->removeAllBlocks();
  occ_layer_->removeAllBlocks();
  resetIncUpdateVars();
  VLOG(1) << "cleared all blocks in all layers.";
}

template <typename T>
void ActMap<T>::prepareInfoFromPointCloud()
{
  CHECK(vis_checker_) << "You need to set visibility checker to compute from "
                         "point cloud.";
  if (!vis_checker_->depthMapPtr())
  {
    LOG(WARNING) << "Vis. checker has no depth map. Did you forgot to set it?";
  }
  if (!vis_checker_->camPtr())
  {
    LOG(WARNING) << "Vis. checker has no camera. Did you forgot to set it?";
  }

  cachePointsAndViewDirs(true);

  info_cal_ptr_.reset(new InfoCalculator(0.1 * pos_factor_layer_->voxel_size(),
                                         0.5, vis_checker_));
}

template <typename T>
void ActMap<T>::cachePointsAndViewDirs(const bool force_update) const
{
  if (!force_update)
  {
    if (!pc_positions_.empty())
    {
      return;
    }
  }

  pc_positions_.clear();
  pc_views_.clear();
  this->getCentersOfOccupiedVoxels(&pc_positions_);
  this->getViewDirsOfOccupiedVoxels(&pc_views_);
  CHECK_EQ(pc_positions_.size(), pc_views_.size());
  CHECK_GT(pc_positions_.size(), 0u) << "need to have some points to work "
                                        "with.";
  VecKVecToEigenKX(pc_positions_, &pc_pos_mat_);

  auto it = pc_views_.begin();
  for (; it != pc_views_.end(); it++)
  {
    if (std::fabs((*it).norm() - 1.0) > 1e-3)
    {
      break;
    }
  }
  if (it != pc_views_.end())
  {
    LOG(WARNING) << "Invalid direction in occupancy voxel, will discard all.";
    pc_views_.clear();
  }
}

// specialization
template <>
void ActMap<QuadTraceVoxel>::getBestViewsAt(
    const size_t cam_id, const int samples_per_side, const bool use_sampling,
    const bool only_updated, Vec3dVec* vox_cs, Vec3dVec* best_views,
    std::vector<double>* values, voxblox::LongIndexVector* global_idxs) const;

template <>
void ActMap<GPTraceVoxel>::getBestViewsAt(
    const size_t cam_id, const int samples_per_side, const bool use_sampling,
    const bool only_updated, Vec3dVec* vox_cs, Vec3dVec* best_views,
    std::vector<double>* values, voxblox::LongIndexVector* global_idxs) const;

template <>
void ActMap<QuadInfoVoxel>::getBestViewFromNearestVoxel(
    const Eigen::Vector3d& pos, const InfoMetricType& type,
    Eigen::Vector3d* vox_cs, Eigen::Vector3d* best_view) const;

template <>
void ActMap<QuadTraceVoxel>::getBestViewFromNearestVoxel(
    const Eigen::Vector3d& pos, const InfoMetricType& type,
    Eigen::Vector3d* vox_cs, Eigen::Vector3d* best_view) const;
template <>
void ActMap<GPInfoVoxel>::getBestViewFromNearestVoxel(
    const Eigen::Vector3d& pos, const InfoMetricType& type,
    Eigen::Vector3d* vox_cs, Eigen::Vector3d* best_view) const;
template <>
void ActMap<GPTraceVoxel>::getBestViewFromNearestVoxel(
    const Eigen::Vector3d& pos, const InfoMetricType& type,
    Eigen::Vector3d* vox_cs, Eigen::Vector3d* best_view) const;

template <>
void ActMap<QuadInfoVoxel>::getFIMFromNearestVoxel(const rpg::Pose& Twc,
                                                   Eigen::Vector3d* vox_c,
                                                   Info* fim) const;

template <>
void ActMap<GPInfoVoxel>::getFIMFromNearestVoxel(const rpg::Pose& Twc,
                                                 Eigen::Vector3d* vox_c,
                                                 Info* fim) const;
}  // namespace act_map
