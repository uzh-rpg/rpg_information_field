#include "act_map/visibility_checker.h"

#include <random>

#include <rpg_common/test_main.h>

using namespace act_map;

class VisCheckerTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    // 2 x 2 cube around the origin
    DepthMapOptions options;
    options.depth_layer_opts_.vox_size = 0.5;
    options.depth_layer_opts_.vox_per_side = 4;
    options.depth_voxel_step_deg_ = 2.0;
    dm_ = std::make_shared<DepthMap>(options);
    dm_->allocateByPoints({ Eigen::Vector3d::Zero() });
    dm_->setLayerConstant(static_cast<float>(uniform_d_));

    cam_ = vi::PinholeCam::createTestCam();

    std::uniform_real_distribution<double> dis_query(-3.0, 3.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    for (size_t i = 0; i < 50; i++)
    {
      rand_query_.emplace_back(
          Eigen::Vector3d(dis_query(gen), dis_query(gen), dis_query(gen)));
    }
  }
  std::random_device rd_;
  rpg::PositionVec rand_query_;

  DepthMapPtr dm_;
  vi::PinholeCamPtr cam_;
  const double uniform_d_ = 2.0;
};

TEST_F(VisCheckerTest, init)
{
  VisibilityCheckerOptions options;
  VisibilityChecker vis_check(options, dm_);

  VisibilityChecker vis_checker_set_later(options);
  vis_checker_set_later.depthMapPtr() = dm_;

  VisibilityChecker vis_checker_with_cam(options, dm_, cam_);
  vis_checker_with_cam.camPtr() = nullptr;
  vis_checker_with_cam.camPtr() = cam_;
}

TEST_F(VisCheckerTest, getVisStatus)
{
  VisibilityCheckerOptions options;
  VisibilityChecker vis_check(options, dm_);

  std::uniform_real_distribution<double> dis_org(-2.0, 2.0);
  std::mt19937 gen(rd_());

  for (size_t i = 0; i < 50; i++)
  {
    const Eigen::Vector3d org(dis_org(gen), dis_org(gen), dis_org(gen));

    {
      VisStatusVec vis_checker_res, dm_res;
      dm_->queryPointsVisibilityAt(org, rand_query_, &dm_res);
      vis_check.getPointsVisibilityAt(org, rand_query_, &vis_checker_res);
      for (size_t j = 0; j < vis_checker_res.size(); j++)
      {
        EXPECT_EQ(vis_checker_res[j], dm_res[j]);
      }
    }

    {
      VisibilityChecker vis_cheker_no_dim(options);
      VisStatusVec vis_status_no_dim;
      vis_cheker_no_dim.getPointsVisibilityAt(org, rand_query_,
                                              &vis_status_no_dim);
      for (size_t j = 0; j < vis_status_no_dim.size(); j++)
      {
        EXPECT_EQ(vis_status_no_dim[j], VisStatus::kUnknownDepth);
      }
    }
  }
}

TEST_F(VisCheckerTest, getVisIdx)
{
  const Eigen::Vector3d org = Eigen::Vector3d::Zero();
  std::set<size_t> vis_idx_vox;
  const DepthVoxel* vox = dm_->depthLayerPtr()->getVoxelPtrByCoordinates(org);
  EXPECT_TRUE(vox != nullptr);
  vox->getVisibleIdxFromPoints(rand_query_, &vis_idx_vox);
  const Eigen::Vector3d vox_c = vox->center();

  // default options
  {
    std::set<size_t> vis_idx;
    VisibilityCheckerOptions def_opts;
    VisibilityChecker def_vis_checker(def_opts, dm_);
    def_vis_checker.getVisibleIdx(org, rand_query_, {}, &vis_idx);
    EXPECT_EQ(vis_idx, vis_idx_vox);
  }

  // depth constraint
  {
    std::set<size_t> vis_idx;
    VisibilityCheckerOptions depth_opts;
    depth_opts.min_dist = uniform_d_ * 0.2;
    depth_opts.max_dist = uniform_d_ * 0.8;
    VisibilityChecker vis_checker_narrow_d(depth_opts, dm_);
    vis_checker_narrow_d.getVisibleIdx(org, rand_query_, {}, &vis_idx);
    EXPECT_LT(vis_idx.size(), vis_idx_vox.size());
    for (const size_t v : vis_idx)
    {
      EXPECT_TRUE(vis_idx_vox.find(v) != vis_idx_vox.end());
    }
  }

  // depth constraint: same as depth map
  {
    std::set<size_t> vis_idx;
    VisibilityCheckerOptions depth_opts;
    depth_opts.max_dist = uniform_d_;
    VisibilityChecker vis_checker_no_dm(depth_opts);
    vis_checker_no_dm.getVisibleIdx(vox_c, rand_query_, {}, &vis_idx);
    EXPECT_EQ(vis_idx, vis_idx_vox);
  }

  // literally no check
  {
    std::set<size_t> vis_idx;
    VisibilityCheckerOptions depth_opts;
    VisibilityChecker vis_checker_no_dm(depth_opts);
    vis_checker_no_dm.getVisibleIdx(vox_c, rand_query_, {}, &vis_idx);
    EXPECT_EQ(vis_idx.size(), rand_query_.size());
  }
}

TEST_F(VisCheckerTest, getVisIdxViewDir)
{
  std::mt19937 gen(rd_());

  std::uniform_real_distribution<double> z_dist(1.0, 3.0);

  VisibilityCheckerOptions angle_opts;
  constexpr double max_angle_rad = 45.0 / 180.0 * M_PI;
  angle_opts.use_view_filtering = true;
  angle_opts.min_view_angle_cos = std::cos(max_angle_rad);

  const double xy_max_dist_ratio = std::tan(max_angle_rad);
  std::uniform_real_distribution<double> in_ratio(0.01,
                                                  xy_max_dist_ratio - 0.01);
  std::uniform_real_distribution<double> out_ratio(xy_max_dist_ratio + 0.01,
                                                   10.0);

  VisibilityChecker vis_checker_angle(angle_opts);

  for (size_t i = 0; i < 50; i++)
  {
    Eigen::Vector3d pt;
    pt.setRandom();
    pt.z() = z_dist(gen);
    Vec3dVec points{ pt };
    const double pt_z = pt.z();
    Eigen::Vector3d view_dir(0.0, 0.0, -1.0);
    Vec3dVec view_dirs{ view_dir };

    {
      Eigen::Vector2d in_dist;
      in_dist.setRandom();
      in_dist = in_dist / in_dist.norm() * pt_z * in_ratio(gen);
      Eigen::Vector3d in_view(in_dist.x() + pt.x(), in_dist.y() + pt.y(), 0.0);
      std::set<size_t> vis_idx;
      vis_checker_angle.getVisibleIdx(in_view, points, view_dirs, &vis_idx);
      EXPECT_EQ(vis_idx.size(), 1u);
      EXPECT_EQ(*std::next(vis_idx.begin(), 0), 0);
    }

    {
      Eigen::Vector2d out_dist;
      out_dist.setRandom();
      out_dist = out_dist / out_dist.norm() * pt_z * out_ratio(gen);
      Eigen::Vector3d out_view(out_dist.x() + pt.x(), out_dist.y() + pt.y(),
                               0.0);
      std::set<size_t> vis_idx;
      vis_checker_angle.getVisibleIdx(out_view, points, view_dirs, &vis_idx);
      EXPECT_EQ(vis_idx.size(), 0);
    }
  }
}

TEST_F(VisCheckerTest, getVisIdxCam)
{
  std::mt19937 gen(rd_());
  constexpr double max_d = 1;
  std::uniform_real_distribution<double> range(-max_d, max_d);

  constexpr size_t kNzPlane_near = 10;
  constexpr size_t kNxPlane_near = 8;
  constexpr size_t kNyPlane_far = 5;
  const double dist_in =
      max_d * std::sqrt(2.1) / (0.5 * cam_->h()) * cam_->fy();
  Vec3dVec points_w;
  for (size_t i = 0; i < kNzPlane_near; i++)
  {
    points_w.emplace_back(range(gen), range(gen), dist_in);
  }
  for (size_t i = 0; i < kNxPlane_near; i++)
  {
    points_w.emplace_back(dist_in, range(gen), range(gen));
  }
  for (size_t i = 0; i < kNyPlane_far; i++)
  {
    points_w.emplace_back(range(gen), dist_in * 10.0, range(gen));
  }

  VisibilityCheckerOptions options;
  VisibilityChecker checker(options, dm_);
  dm_->setLayerConstant(static_cast<float>(dist_in * 5.0));

  VisIdx vis_idx;
  checker.getVisibleIdx(Eigen::Vector3d::Zero(), points_w, {}, &vis_idx);
  EXPECT_EQ(kNzPlane_near + kNxPlane_near, vis_idx.size());
  rpg::Pose Twb;
  Twb.setIdentity();
  checker.getVisibleIdx(Twb * cam_->Tbc(), points_w, {}, &vis_idx);
  EXPECT_EQ(kNzPlane_near + kNxPlane_near, vis_idx.size());

  Twb.setIdentity();
  checker.camPtr() = cam_;
  checker.getVisibleIdx(Twb * cam_->Tbc(), points_w, {}, &vis_idx);
  EXPECT_EQ(kNzPlane_near, vis_idx.size());

  Twb.getRotation() = rpg::Rotation(Eigen::Vector3d(0.0, M_PI * 0.5, 0.0));
  checker.getVisibleIdx(Twb * cam_->Tbc(), points_w, {}, &vis_idx);
  EXPECT_EQ(kNxPlane_near, vis_idx.size());

  Twb.getRotation() = rpg::Rotation(Eigen::Vector3d(- M_PI * 0.5, 0.0, 0.0));
  checker.getVisibleIdx(Twb * cam_->Tbc(), points_w, {}, &vis_idx);
  EXPECT_EQ(0u, vis_idx.size());

  dm_->setLayerConstant(static_cast<float>(dist_in * 60));
  checker.getVisibleIdx(Twb * cam_->Tbc(), points_w, {}, &vis_idx);
  EXPECT_EQ(kNyPlane_far, vis_idx.size());
}

RPG_COMMON_TEST_MAIN
{
}
