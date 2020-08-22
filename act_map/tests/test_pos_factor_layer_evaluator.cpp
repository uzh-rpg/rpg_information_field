#include "act_map/pos_factor_layer_evaluator.h"

#include <random>

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>
#include <vi_utils/map.h>

#include "act_map/voxblox/io/layer_io.h"
#include "act_map/positional_factor_layer_integrator.h"
#include "act_map/sampler.h"

using namespace act_map;

template <typename T>
class PositionalFactorEvaluatorTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    std::string abs_map = dir + "/test_data/test_map_100.txt";

    vi::Map map;
    map.load(abs_map, std::string());

    QuadVisScoreOptions quad_vis_opt;
    quad_vis_opt.half_fov_rad = M_PI_4;
    quad_vis_opt.boundary_value = 0.9;
    quad_vis_opt.boundary_to_mid_ratio = 0.9;
    QuadVisScorePtr vscore = std::make_shared<QuadraticVisScore>(quad_vis_opt);
    vscore->initSecondOrderApprox(0.9, 0.9);
    k1_ = vscore->k1();
    k2_ = vscore->k2();
    k3_ = vscore->k3();
    setQuadPolyVisiblity(quad_vis_opt);

    std::string vis_dir = dir + "/test_data/fov45_fs30_lm1000_k10_fast";
    setGPVisiblityFromFolder(vis_dir);

    LayerOptions pos_fac_layer_options;
    factor_layer_ptr_ = std::make_shared<voxblox::Layer<T>>(
        pos_fac_layer_options.vox_size, pos_fac_layer_options.vox_per_side);
    rpg::PositionVec uni_points;
    utils::generateUniformPointsWithin(pos_fac_layer_options.vox_size, 2.1, 2.1,
                                       2.1, &uni_points);
    for (const Eigen::Vector3d& pt : uni_points)
    {
      factor_layer_ptr_->allocateBlockPtrByCoordinates(pt);
    }

    LayerOptions occ_options;
    occ_layer_ptr_ = std::make_shared<OccupancyLayer>(occ_options.vox_size,
                                                      occ_options.vox_per_side);
    utils::setPointsInOccupancyLayer(map.points_, occ_layer_ptr_.get());

    PositionalFactorLayerIntegratorOptions options;
    VisibilityCheckerOptions vis_check_options;
    VisibilityCheckerPtr vis_check_ptr(
        new VisibilityChecker(vis_check_options));
    PositionalFactorLayerIntegrator<T> integrator1(
        options, occ_layer_ptr_, factor_layer_ptr_, vis_check_ptr);
    voxblox::BlockIndexList cand_blks;
    factor_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
    integrator1.recomputeFromOccupancyLayer(cand_blks);
  }

  OccupancyLayer::Ptr occ_layer_ptr_;
  typename voxblox::Layer<T>::Ptr factor_layer_ptr_;
  double k1_;
  double k2_;
  double k3_;
};

using PosFactorVoxelTypes =
    ::testing::Types<QuadInfoVoxel, QuadTraceVoxel, GPInfoVoxel, GPTraceVoxel,
                     QuadPolyInfoVoxel, QuadPolyTraceVoxel>;
TYPED_TEST_CASE(PositionalFactorEvaluatorTest, PosFactorVoxelTypes);

TYPED_TEST(PositionalFactorEvaluatorTest, testInit)
{
  PosFactorLayerEvaluator<TypeParam> eval(this->factor_layer_ptr_.get());
  eval.setQuadraticCoefs(this->k1_, this->k2_, this->k3_);

  const voxblox::Layer<TypeParam>* lptr = eval.layerPtr();
  EXPECT_EQ(lptr, this->factor_layer_ptr_.get());
}

TYPED_TEST(PositionalFactorEvaluatorTest, testNearestRead)
{
  PosFactorLayerEvaluator<TypeParam> eval(this->factor_layer_ptr_.get());
  eval.setQuadraticCoefs(this->k1_, this->k2_, this->k3_);
  Eigen::Vector3d rpt;
  rpt.setRandom();
  Eigen::Vector3d far(100, 100, 100);

  const TypeParam* tvox_ptr = eval.getVoxelNearest(rpt);
  const TypeParam* far_tvox_ptr = eval.getVoxelNearest(far);
  EXPECT_TRUE(far_tvox_ptr == nullptr);
  EXPECT_TRUE(tvox_ptr != nullptr);

  typename voxblox::Block<TypeParam>::Ptr blk_ptr =
      this->factor_layer_ptr_->getBlockPtrByCoordinates(rpt);
  EXPECT_TRUE(blk_ptr != nullptr);
  const TypeParam& tvox = blk_ptr->getVoxelByCoordinates(rpt);
  EXPECT_TRUE(voxblox::utils::isSameVoxel(*tvox_ptr, tvox));
}

TYPED_TEST(PositionalFactorEvaluatorTest, testGetValue)
{
  PosFactorLayerEvaluator<TypeParam> eval(this->factor_layer_ptr_.get());
  eval.setQuadraticCoefs(this->k1_, this->k2_, this->k3_);
  Eigen::Vector3d rpt;
  rpt.setRandom();
  rpt *= 0.5;
  rpg::Pose rpose;
  rpose.getPosition() = rpt;
  rpose.getRotation().setRandom();

  std::vector<InfoMetricType> test_types{ InfoMetricType::kDet,
                                          InfoMetricType::kTrace,
                                          InfoMetricType::kMinEig };
  for (const InfoMetricType& v : test_types)
  {
    double val;
    bool suc = eval.getValueInterpolation(rpose, v, &val);
    EXPECT_TRUE(suc);
  }
}

TYPED_TEST(PositionalFactorEvaluatorTest, testInterpolation)
{
  PosFactorLayerEvaluator<TypeParam> eval(this->factor_layer_ptr_.get());
  eval.setQuadraticCoefs(this->k1_, this->k2_, this->k3_);

  voxblox::BlockIndexList blk_idxs;
  this->factor_layer_ptr_->getAllAllocatedBlocks(&blk_idxs);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<size_t> dis_blk_idx(0, blk_idxs.size() - 1);
  std::uniform_real_distribution<double> rdist_idx_near(0.05, 0.15);
  std::uniform_real_distribution<double> rdist_idx_far(0.20, 0.3);

  typename voxblox::Block<TypeParam>::Ptr blk_ptr =
      this->factor_layer_ptr_->getBlockPtrByIndex(blk_idxs[dis_blk_idx(gen)]);
  EXPECT_TRUE(blk_ptr.get() != nullptr);

  const size_t lin_idx = blk_ptr->num_voxels() / 2;
  const double vox_size = blk_ptr->voxel_size();
  Eigen::Vector3d vox_c = blk_ptr->computeCoordinatesFromLinearIndex(lin_idx);

  Eigen::Vector3d vox_near = vox_c.array() + rdist_idx_near(gen) * vox_size;
  Eigen::Vector3d vox_far = vox_c.array() + rdist_idx_far(gen) * vox_size;

  std::vector<InfoMetricType> test_types{ InfoMetricType::kDet,
                                          InfoMetricType::kTrace,
                                          InfoMetricType::kMinEig };
  for (const InfoMetricType& v : test_types)
  {
    rpg::Pose Twc;
    Twc.setRandom();
    bool res = false;

    Twc.getPosition() = vox_c;
    double val_c = 0.0;
    res = eval.getValueInterpolation(Twc, v, &val_c);
    EXPECT_TRUE(res);
    double val_c_no_interp = 0;
    res = eval.getValueNearest(Twc, v, &val_c_no_interp);
    EXPECT_TRUE(res);
    EXPECT_DOUBLE_EQ(val_c, val_c_no_interp);

    Twc.getPosition() = vox_near;
    double val_near = 0.0;
    res = eval.getValueInterpolation(Twc, v, &val_near);
    EXPECT_TRUE(res);

    Twc.getPosition() = vox_far;
    double val_far = 0.0;
    res = eval.getValueInterpolation(Twc, v, &val_far);
    EXPECT_TRUE(res);

    EXPECT_GT(std::abs(val_c - val_far), std::abs(val_c - val_near));
  }
}

TYPED_TEST(PositionalFactorEvaluatorTest, testGetGradient)
{
  PosFactorLayerEvaluator<TypeParam> eval(this->factor_layer_ptr_.get());
  eval.setQuadraticCoefs(this->k1_, this->k2_, this->k3_);

  voxblox::BlockIndexList blk_idxs;
  this->factor_layer_ptr_->getAllAllocatedBlocks(&blk_idxs);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<size_t> dis_blk_idx(0, blk_idxs.size() - 1);
  std::uniform_real_distribution<double> rdist_idx_near(-0.1, 0.1);

  typename voxblox::Block<TypeParam>::Ptr blk_ptr =
      this->factor_layer_ptr_->getBlockPtrByIndex(blk_idxs[dis_blk_idx(gen)]);
  EXPECT_TRUE(blk_ptr.get() != nullptr);

  voxblox::VoxelIndex vox_idx;
  vox_idx.setConstant(blk_ptr->voxels_per_side() / 2);
  const double vox_size = blk_ptr->voxel_size();
  Eigen::Vector3d vox_c = blk_ptr->computeCoordinatesFromVoxelIndex(vox_idx);

  std::vector<InfoMetricType> test_types{ InfoMetricType::kDet,
                                          InfoMetricType::kTrace,
                                          InfoMetricType::kMinEig };
  rpg::Pose Twc;
  Twc.setRandom();
  Twc.getPosition() = vox_c;
  // get gradient nearest
  for (const InfoMetricType& v : test_types)
  {
    bool res = true;
    double val_no_interp;
    Eigen::Vector3d dpos_no_interp;
    res =
        eval.getValueNearest(Twc, v, &val_no_interp, &dpos_no_interp, nullptr);
    ASSERT_TRUE(res);

    // step = voxel size: two methods should give the same results
    eval.setNumericalStepRatio(1.0);
    double val_interp;
    Eigen::Vector3d dpos_interp;
    res =
        eval.getValueInterpolation(Twc, v, &val_interp, &dpos_interp, nullptr);
    ASSERT_TRUE(res);
    EXPECT_EQ(val_interp, val_no_interp);
    EXPECT_TRUE(dpos_interp.isApprox(dpos_no_interp));

    eval.setNumericalStepRatio(0.1);
    res =
        eval.getValueInterpolation(Twc, v, &val_interp, &dpos_interp, nullptr);
    EXPECT_EQ(val_interp, val_no_interp);

    for (int i = 0; i < 3; i++)
    {
      rpg::Pose Twc_m = Twc;
      rpg::Pose Twc_p = Twc;
      (Twc_m.getPosition()(i)) -= vox_size;
      (Twc_p.getPosition()(i)) += vox_size;

      double val_m, val_p;
      eval.getValueNearest(Twc_m, v, &val_m);
      eval.getValueNearest(Twc_p, v, &val_p);

      EXPECT_DOUBLE_EQ(val_p - val_m, dpos_no_interp(i) * 2 * vox_size);
      EXPECT_GT(dpos_interp(i) * dpos_no_interp(i), 0.0);
    }
  }
}

RPG_COMMON_TEST_MAIN
{
}
