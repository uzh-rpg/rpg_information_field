#include "act_map/common.h"

#include <random>

#include <rpg_common/main.h>
#include <rpg_common/save.h>
#include <vi_utils/map.h>

#include "act_map/voxblox_utils.h"
#include "act_map/positional_factor_layer_integrator.h"
#include "act_map/pos_factor_layer_evaluator.h"

DEFINE_string(abs_map, "", "absolute path of the map file.");
DEFINE_string(abs_trace_dir, "", "trace dir to save results");
DEFINE_string(abs_gp_vis_dir, "", "directory of GP visibility profile");

// ranges to allocate the blocks
DEFINE_double(xrange, 5.0, "X range centered at zero.");
DEFINE_double(yrange, 5.0, "Y range centered at zero.");
DEFINE_double(zrange, 2.0, "Z range centered at zero.");

// distance step to sample positions for testing gradient
DEFINE_double(pos_sample_step, 0.5, "Step to sample the positions.");

// sample `n_pos_samples_for_rot` positions, and at each position
// sample `n_rot_samples` for rotation
DEFINE_uint32(n_rot_samples, 10u, "Number of rotation samples to test.");
DEFINE_uint32(n_pos_samples_for_rot, 4u, "Number of positions samples to test "
                                         "different rotations");
//
DEFINE_double(hfov_deg, 45.0, "half FoV");

using namespace act_map;

// structure to hold the gradient test result
struct GradRes
{
  GradRes(const std::string& name, const size_t n_pos,
          const size_t n_pos_for_rot, const size_t n_rot)
    : name_(name)
    , test_positions_(n_pos)
    , test_pos_bearings_w_(n_pos)
    , pos_grads_(n_pos)
    , rot_test_positions_(n_pos_for_rot)
    , test_bearings_w_(n_rot)
    , rot_grads_(n_pos_for_rot)
    , drot_bearing_w_(n_pos_for_rot)
  {
    for (size_t i = 0; i < n_pos_for_rot; i++)
    {
      rot_grads_.at(i).resize(n_rot);
      drot_bearing_w_.at(i).resize(n_rot);
    }
  }

  GradRes()
  {
  }

  void resize(const size_t n_pos, const size_t n_pos_for_rot,
              const size_t n_rot)
  {
    test_positions_.resize(n_pos);
    test_pos_bearings_w_.resize(n_pos);
    pos_grads_.resize(n_pos);

    rot_test_positions_.resize(n_pos_for_rot);
    test_bearings_w_.resize(n_rot);
    rot_grads_.resize(n_pos_for_rot);
    drot_bearing_w_.resize(n_pos_for_rot);
    for (size_t i = 0; i < n_pos_for_rot; i++)
    {
      rot_grads_.at(i).resize(n_rot);
      drot_bearing_w_.at(i).resize(n_rot);
    }
  }

  void save(const std::string& abs_save_dir) const
  {
    const std::string save_prefix = abs_save_dir + "/" + name_ + "_";

    // positions and gradients
    Eigen::MatrixX3d test_pos_mat;
    act_map::VecKVecToEigenXK(test_positions_, &test_pos_mat);
    rpg::save(save_prefix + "test_positions.txt", test_pos_mat);

    Eigen::MatrixX3d test_pos_bearing_mat;
    act_map::VecKVecToEigenXK(test_pos_bearings_w_, &test_pos_bearing_mat);
    rpg::save(save_prefix + "test_positions_bearings_w.txt",
              test_pos_bearing_mat);

    Eigen::MatrixX3d pos_grads_mat;
    act_map::VecKVecToEigenXK(pos_grads_, &pos_grads_mat);
    rpg::save(save_prefix + "pos_grad.txt", pos_grads_mat);

    // rotation and gradients
    Eigen::MatrixX3d rot_test_pos_mat;
    act_map::VecKVecToEigenXK(rot_test_positions_, &rot_test_pos_mat);
    rpg::save(save_prefix + "rot_test_positions.txt", rot_test_pos_mat);

    Eigen::MatrixX3d test_bearings_mat;
    act_map::VecKVecToEigenXK(test_bearings_w_, &test_bearings_mat);
    rpg::save(save_prefix + "test_bearings_w.txt", test_bearings_mat);

    CHECK_EQ(rot_grads_.size(), rot_test_positions_.size());
    CHECK_EQ(drot_bearing_w_.size(), rot_test_positions_.size());
    for (size_t pos_i = 0; pos_i < rot_test_positions_.size(); pos_i++)
    {
      Eigen::MatrixX3d rot_grad_mat;
      act_map::VecKVecToEigenXK(rot_grads_[pos_i], &rot_grad_mat);
      rpg::save(save_prefix + "rot_grad" + std::to_string(pos_i) + ".txt",
                rot_grad_mat);
      Eigen::MatrixX3d rot_new_bearing_mat;
      act_map::VecKVecToEigenXK(drot_bearing_w_[pos_i], &rot_new_bearing_mat);
      rpg::save(save_prefix + "drot_bearing_w" + std::to_string(pos_i) + ".txt",
                rot_new_bearing_mat);
    }
  }

  std::string name_;

  // positions and corresponding gradients
  act_map::Vec3dVec test_positions_;
  act_map::Vec3dVec test_pos_bearings_w_;
  act_map::Vec3dVec pos_grads_;

  // rotation and corresponding gradient
  act_map::Vec3dVec rot_test_positions_;
  act_map::Vec3dVec test_bearings_w_;
  act_map::V3dVecVec rot_grads_;
  // the new bearing after update the rotation using rot_grads
  act_map::V3dVecVec drot_bearing_w_;
};

namespace
{
template <typename VoxT>
typename voxblox::Layer<VoxT>::Ptr makeKernelLayer(
    const rpg::PositionVec& positions, const OccupancyLayer::Ptr& occ_layer_ptr)
{
  LayerOptions ker_options;
  ker_options.vox_size = 0.1;
  typename voxblox::Layer<VoxT>::Ptr pos_factor_layer_ptr =
      std::make_shared<voxblox::Layer<VoxT>>(ker_options.vox_size,
                                             ker_options.vox_per_side);
  for (const Eigen::Vector3d& pt : positions)
  {
    pos_factor_layer_ptr->allocateBlockPtrByCoordinates(pt);
  }

  PositionalFactorLayerIntegratorOptions options;
  VisibilityCheckerOptions vis_options;
  VisibilityCheckerPtr vis_check_ptr(new VisibilityChecker(vis_options));
  PositionalFactorLayerIntegrator<VoxT> integrator(options, occ_layer_ptr, pos_factor_layer_ptr,
                                         vis_check_ptr);
  voxblox::BlockIndexList cand_blks;
  pos_factor_layer_ptr->getAllAllocatedBlocks(&cand_blks);
  integrator.recomputeFromOccupancyLayer(cand_blks);
  return pos_factor_layer_ptr;
}

template <typename VoxT>
void evaluate(const voxblox::Layer<VoxT>& pos_factor_layer_ptr,
              const InfoMetricType& info_t, const std::string& name,
              const QuadraticVisScore& vis_score,
              const rpg::PositionVec& test_positions,
              const rpg::PositionVec& positions_to_test_rotations,
              const rpg::RotationVec& test_rotations, GradRes* res)
{
  CHECK_NOTNULL(res);

  res->name_ = name;
  res->resize(test_positions.size(), positions_to_test_rotations.size(),
              test_rotations.size());

  PosFactorLayerEvaluator<VoxT> eval(&pos_factor_layer_ptr);
  eval.setQuadraticCoefs(vis_score.k1(), vis_score.k2(), vis_score.k3());

  VLOG(1) << "Evaluating position gradient...";
  // gradients with respect to position
  rpg::Rotation pos_rot;
  pos_rot.setRandom();
  const Eigen::Vector3d test_pos_bearing_w =
      pos_rot.rotate(Eigen::Vector3d(0, 0, 1.0));
  for (size_t pos_i = 0; pos_i < test_positions.size(); pos_i++)
  {
    rpg::Pose T_wc(pos_rot, test_positions[pos_i]);
    double val;
    Eigen::Vector3d dpos;
    eval.getValueInterpolation(T_wc, info_t, &val, &dpos);

    res->test_positions_[pos_i] = test_positions[pos_i];
    res->test_pos_bearings_w_[pos_i] = test_pos_bearing_w;
    res->pos_grads_[pos_i] = dpos;
  }

  VLOG(1) << "Evaluating rotation gradient...";
  // save the bearings for visualization
  const Eigen::Vector3d z_cam(0.0, 0.0, 1.0);
  for (size_t rot_i = 0; rot_i < test_rotations.size(); rot_i++)
  {
    res->test_bearings_w_[rot_i] = test_rotations[rot_i].rotate(z_cam);
  }
  // gradients with respect to orientations
  VLOG(1) << "- raw gradient and normalizaiton";
  V3dVecVec normalized_rot_grad_at_pos;
  normalized_rot_grad_at_pos.resize(positions_to_test_rotations.size());
  constexpr double normalized_max_rad = (10.0 / 180.0) * M_PI;
  for (size_t pos_i = 0; pos_i < positions_to_test_rotations.size(); pos_i++)
  {
    const rpg::Position& cur_test_pos = positions_to_test_rotations[pos_i];
    res->rot_test_positions_[pos_i] = cur_test_pos;
    Vec3dVec& cur_grad_res = res->rot_grads_[pos_i];
    double cur_max_rad = -1;
    for (size_t rot_i = 0; rot_i < test_rotations.size(); rot_i++)
    {
      rpg::Pose T_wc(test_rotations[rot_i], cur_test_pos);
      double val;
      Eigen::Vector3d drot_g;
      eval.getValueInterpolation(T_wc, info_t, &val, nullptr, &drot_g);
      cur_grad_res[rot_i] = drot_g;
      if (drot_g.norm() > cur_max_rad)
      {
        cur_max_rad = drot_g.norm();
      }
    }
    const double normalize_factor = normalized_max_rad / cur_max_rad;
    Vec3dVec& cur_norm_grad = normalized_rot_grad_at_pos[pos_i];
    cur_norm_grad.resize(test_rotations.size());
    for (size_t rot_i = 0; rot_i < test_rotations.size(); rot_i++)
    {
      cur_norm_grad[rot_i] = cur_grad_res[rot_i] * normalize_factor;
    }
  }
  // one step gradient ascent
  VLOG(1) << "- one step gradient descent";
  for (size_t pos_i = 0; pos_i < positions_to_test_rotations.size(); pos_i++)
  {
    Vec3dVec& cur_drot_new_bearing = res->drot_bearing_w_[pos_i];
    const Vec3dVec& cur_n_rot_grad = normalized_rot_grad_at_pos[pos_i];
    for (size_t rot_i = 0; rot_i < test_rotations.size(); rot_i++)
    {
      rpg::Rotation new_rot =
          rpg::Rotation::exp(cur_n_rot_grad[rot_i]) * test_rotations[rot_i];
      cur_drot_new_bearing[rot_i] = new_rot.rotate(z_cam);
    }
  }
}
}

RPG_COMMON_MAIN
{
  CHECK(!FLAGS_abs_map.empty());
  CHECK(!FLAGS_abs_trace_dir.empty());

  std::cout << "This is to test and visualize the gradient of sampled 6 DoF"
               " pose in different scenarios for qualitative check."
            << std::endl;

  std::cout << "Experiment parameters:\n"
            << "- abs_map: " << FLAGS_abs_map << std::endl
            << "- abs_trace_dir: " << FLAGS_abs_trace_dir << std::endl
            << "- abs_gp_vis_dir: " << FLAGS_abs_gp_vis_dir << std::endl
            << "- Layer ranges centered at zero:\n"
            << " - xrange: " << FLAGS_xrange << std::endl
            << " - yrange: " << FLAGS_yrange << std::endl
            << " - zrange: " << FLAGS_zrange << std::endl
            << " - pos sample step: " << FLAGS_pos_sample_step << std::endl
            << " - # rot samples: " << FLAGS_n_rot_samples << std::endl
            << " - # pos sample for rotation test: "
            << FLAGS_n_pos_samples_for_rot << std::endl
            << "- Half FoV degree: " << FLAGS_hfov_deg << std::endl;

  std::random_device rd;
  std::mt19937 rng(rd());

  VLOG(1) << "Initialize visibility...";
  VisApproxPtr<GPVisApprox> gp_vis_ptr =
      std::make_shared<GPVisibilityApproximator>();
  gp_vis_ptr->load(FLAGS_abs_gp_vis_dir);
  GPInfoVoxel::setVisApprox(gp_vis_ptr);
  GPTraceVoxel::setVisApprox(gp_vis_ptr);
  double hfov_rad = FLAGS_hfov_deg * M_PI / 180.0;
  QuadraticVisScore vscore(hfov_rad);
  vscore.initSecondOrderApprox(0.5, 0.5);

  VLOG(1) << "Set map points in the occupancy layer...";
  vi_utils::Map map;
  map.load(FLAGS_abs_map, std::string());
  LayerOptions occ_options;
  occ_options.vox_size = 0.05;
  OccupancyLayer::Ptr occ_layer_ptr = std::make_shared<OccupancyLayer>(
      occ_options.vox_size, occ_options.vox_per_side);
  utils::setPointsInOccupancyLayer(map.points_, occ_layer_ptr.get());
  VLOG(1) << "- Loaded " << map.n_points_ << " map points.";
  VLOG(1) << "- Set " << occ_layer_ptr->getNumberOfAllocatedBlocks()
          << " blocks in the occupancy layer.";

  VLOG(1) << "Sample positions to evaluate...";
  rpg::PositionVec test_positions;
  utils::generateUniformPointsWithin(FLAGS_pos_sample_step, FLAGS_xrange,
                                     FLAGS_yrange, FLAGS_zrange,
                                     &test_positions);
  VLOG(1) << "- Sampled " << test_positions.size() << " positions.";

  VLOG(1) << "Sample rotations to evaluate...";
  rpg::RotationVec dense_rot;
  utils::sampleRotation(1, &dense_rot);
  rpg::RotationVec test_rotations;
  std::vector<size_t> rand_rot_indices;
  utils::sampleIndicesInRange(FLAGS_n_rot_samples, 0, dense_rot.size() - 1,
                              &rand_rot_indices);
  CHECK_EQ(rand_rot_indices.size(), FLAGS_n_rot_samples);
  for (size_t i = 0; i < FLAGS_n_rot_samples; i++)
  {
    test_rotations.push_back(dense_rot[rand_rot_indices[i]]);
  }
  VLOG(1) << "- Sampled " << test_rotations.size() << " rotations.";

  // Positions at which random rotations will be sampled and tested
  rpg::PositionVec positions_for_test_rot;
  std::vector<size_t> pos_for_rot_indices;
  utils::sampleIndicesInRange(FLAGS_n_pos_samples_for_rot, 0,
                              test_positions.size() - 1, &pos_for_rot_indices);
  CHECK_EQ(pos_for_rot_indices.size(), FLAGS_n_pos_samples_for_rot);
  for (size_t i = 0; i < FLAGS_n_pos_samples_for_rot; i++)
  {
    positions_for_test_rot.push_back(test_positions.at(pos_for_rot_indices[i]));
  }
  VLOG(1) << "- which will be tested at " << positions_for_test_rot.size()
          << " positions.";

  rpg::Timer timer;
  VLOG(1) << "Test for different voxel types...";
  timer.start();
  {
    VLOG(1) << ">>> GPTraceLayer - trace";
    VLOG(1) << "Creating kernel layer...";
    GPTraceLayer::Ptr gp_trace_layer =
        makeKernelLayer<GPTraceVoxel>(test_positions, occ_layer_ptr);
    VLOG(1) << "Evaluating gradients...";
    GradRes res_gp_trace;
    evaluate(*gp_trace_layer, InfoMetricType::kTrace, std::string("gp_trace"),
             vscore, test_positions, positions_for_test_rot, test_rotations,
             &res_gp_trace);
    VLOG(1) << "Saving...";
    res_gp_trace.save(FLAGS_abs_trace_dir);
  }
  VLOG(1) << "<<< Time: " << timer.stop() << " seconds.";

  timer.start();
  {
    VLOG(1) << ">>> GPInfoLayer - Determinant";
    VLOG(1) << "Creating kernel layer...";
    GPInfoLayer::Ptr gp_info_layer =
        makeKernelLayer<GPInfoVoxel>(test_positions, occ_layer_ptr);
    VLOG(1) << "Evaluating gradients...";
    GradRes res_gp_info_det;
    evaluate(*gp_info_layer, InfoMetricType::kDet, std::string("gp_info_det"),
             vscore, test_positions, positions_for_test_rot, test_rotations,
             &res_gp_info_det);
    VLOG(1) << "Saving...";
    res_gp_info_det.save(FLAGS_abs_trace_dir);
  }
  VLOG(1) << "<<< Time: " << timer.stop() << " seconds.";

  timer.start();
  {
    VLOG(1) << ">>> GPInfoLayer - MinEig";
    VLOG(1) << "Creating kernel layer...";
    GPInfoLayer::Ptr gp_info_layer =
        makeKernelLayer<GPInfoVoxel>(test_positions, occ_layer_ptr);
    VLOG(1) << "Evaluating gradients...";
    GradRes res_gp_info_meig;
    evaluate(*gp_info_layer, InfoMetricType::kMinEig,
             std::string("gp_info_meig"), vscore, test_positions,
             positions_for_test_rot, test_rotations, &res_gp_info_meig);
    VLOG(1) << "Saving...";
    res_gp_info_meig.save(FLAGS_abs_trace_dir);
  }
  VLOG(1) << "<<< Time: " << timer.stop() << " seconds.";

  timer.start();
  {
    VLOG(1) << ">>> Quadradtic - Det";
    VLOG(1) << "Creating kernel layer...";
    QuadInfoLayer::Ptr quad_info_layer =
        makeKernelLayer<QuadInfoVoxel>(test_positions, occ_layer_ptr);
    VLOG(1) << "Evaluating gradients...";
    GradRes res_quad_info_meig;
    evaluate(*quad_info_layer, InfoMetricType::kMinEig,
             std::string("quad_info_det"), vscore, test_positions,
             positions_for_test_rot, test_rotations, &res_quad_info_meig);
    VLOG(1) << "Saving...";
    res_quad_info_meig.save(FLAGS_abs_trace_dir);
  }
  VLOG(1) << "<<< Time: " << timer.stop() << " seconds.";

  timer.start();
  {
    VLOG(1) << ">>> Quadradtic - Trace";
    VLOG(1) << "Creating kernel layer...";
    QuadTraceLayer::Ptr quad_trace_layer =
        makeKernelLayer<QuadTraceVoxel>(test_positions, occ_layer_ptr);
    VLOG(1) << "Evaluating gradients...";
    GradRes res_quad_trace;
    evaluate(*quad_trace_layer, InfoMetricType::kTrace,
             std::string("quad_trace"), vscore, test_positions,
             positions_for_test_rot, test_rotations, &res_quad_trace);
    VLOG(1) << "Saving...";
    res_quad_trace.save(FLAGS_abs_trace_dir);
  }

  //  timer.start();
  return 0;
}
