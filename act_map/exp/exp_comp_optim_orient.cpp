//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/optim_orient.h"

#include <rpg_common/main.h>
#include <rpg_common/timer.h>
#include <vi_utils/cam_min.h>

#include "act_map/sampler.h"
#include "act_map/vis_score.h"

DEFINE_string(abs_trace_dir, "", "trace dir to save results");

DEFINE_int32(n_trials, 50, "Number of trials");
DEFINE_int32(n_pts, 500, "Number of random points each trial.");
DEFINE_int32(n_positions, 50, "Number of random positions each trial.");

DEFINE_double(rdn_pts_size, 10, "Size of the cube for random points.");
DEFINE_double(rdn_pos_size, 5, "Size of the cube for random points.");

DEFINE_double(pts_rad_low, 0.8, "Lower bound radius.");
DEFINE_double(pts_rad_high, 1.2, "High bound radius.");

DEFINE_double(angle_res_deg, 10.0, "Resolution to sample points on a sphere.");

using namespace act_map;
using namespace act_map::optim_orient;

RPG_COMMON_MAIN
{
  CHECK(!FLAGS_abs_trace_dir.empty());

  std::cout << "This is to compare the determined optimal orientation based "
               "different metric, from both approximated information and "
               "the exact information.";

  std::cout << "Experiment parameters:\n"
            << "- abs_trace_dir: " << FLAGS_abs_trace_dir << std::endl
            << "- n_trials: " << FLAGS_n_trials << std::endl
            << "- n_rdn_poitns: " << FLAGS_n_pts << std::endl
            << "- n_rdn_positions: " << FLAGS_n_positions << std::endl
            << "- rdn_pts_size: " << FLAGS_rdn_pts_size << std::endl
            << "- rdn_position_size: " << FLAGS_rdn_pos_size << std::endl
            << "- angle_res_deg: " << FLAGS_angle_res_deg << std::endl;

  std::cout << "Will generate points from "
            << FLAGS_pts_rad_low* FLAGS_rdn_pts_size << " to "
            << FLAGS_pts_rad_high* FLAGS_rdn_pts_size << std::endl;

  rpg::Pose Tbc;
  Tbc.setIdentity();
  vi::PinholeCam cam({ 300, 300, 300, 300, 600, 600 }, Tbc);
  VLOG(1) << "Simulated a camera:\n" << cam;

  double hfov_rad = M_PI_4;
  VisScore vscore(hfov_rad);
  vscore.initSecondOrderApprox(0.5, 0.5);

  rpg::RotationVec rot_samples;
  utils::sampleRotation(FLAGS_angle_res_deg, &rot_samples);

  VLOG(1) << "Generating test types and result struct...";
  const int n_all_pos = FLAGS_n_trials * FLAGS_n_positions;
  std::vector<InfoMetricType> test_types{ InfoMetricType::kMinEig,
                                          InfoMetricType::kDet,
                                          InfoMetricType::kTrace };
  std::map<InfoMetricType, OptimOrientRes> res_appr;
  std::map<InfoMetricType, OptimOrientRes> res_exact;
  double time_appr_constr = 0.0;
  std::map<InfoMetricType, double> time_appr_query;
  std::map<InfoMetricType, double> time_exact;
  for (const InfoMetricType v : test_types)
  {
    const std::string nm = kInfoMetricNames[v];
    res_appr.insert({ v, OptimOrientRes(n_all_pos, nm + "_app") });
    res_exact.insert({ v, OptimOrientRes(n_all_pos, nm + "_exact") });
    time_appr_query.insert({ v, 0 });
    time_exact.insert({ v, 0 });
  }

  int pos_cnt = 0;
  rpg::Timer timer;
  for (int trial_i = 0; trial_i < FLAGS_n_trials; trial_i++)
  {
    std::cout << "\nTrial " << trial_i + 1 << "/" << FLAGS_n_trials
              << std::endl;
    Eigen::Matrix3Xd points;
    //    utils::generateRandomPointsWithin(FLAGS_n_pts,
    //                                      -FLAGS_rdn_pts_size / 2,
    //                                      FLAGS_rdn_pts_size / 2,
    //                                      -FLAGS_rdn_pts_size / 2,
    //                                      FLAGS_rdn_pts_size / 2,
    //                                      -FLAGS_rdn_pts_size / 2,
    //                                      FLAGS_rdn_pts_size / 2,
    //                                      &points);
    utils::generateRandomPointsWithin(FLAGS_n_pts,
                                      FLAGS_pts_rad_low * FLAGS_rdn_pts_size,
                                      FLAGS_pts_rad_high * FLAGS_rdn_pts_size,
                                      &points);
    rpg::PositionVec pos_vec;
    utils::generateRandomPointsWithin(FLAGS_n_positions,
                                      -FLAGS_rdn_pos_size / 2,
                                      FLAGS_rdn_pos_size / 2,
                                      -FLAGS_rdn_pos_size / 2,
                                      FLAGS_rdn_pos_size / 2,
                                      -FLAGS_rdn_pos_size / 2,
                                      FLAGS_rdn_pos_size / 2,
                                      &pos_vec);

    VLOG(1) << "Constructing kernels...";
    InfoK1Vec info_k1_vec(FLAGS_n_positions);
    InfoK2Vec info_k2_vec(FLAGS_n_positions);
    InfoK3Vec info_k3_vec(FLAGS_n_positions);
    timer.start();
    for (int pos_i = 0; pos_i < FLAGS_n_positions; pos_i++)
    {
      constructKernelBatch(points,
                           pos_vec[pos_i],
                           &(info_k1_vec[pos_i]),
                           &(info_k2_vec[pos_i]),
                           &(info_k3_vec[pos_i]));
    }
    time_appr_constr += timer.stop();

    VLOG(1) << "Determing optimal orientation...";
    double cnt_r = 0.0;
    for (int pos_i = 0; pos_i < FLAGS_n_positions; pos_i++)
    {
      for (const InfoMetricType mtype : test_types)
      {
        if (pos_i > cnt_r * FLAGS_n_positions)
        {
          std::cout << "." << std::flush;
          cnt_r += 0.1;
        }
        OptimOrientRes& cur_res_appr = res_appr[mtype];
        timer.start();
        getOptimViewFromInfoKernels(rot_samples,
                                    vscore.k1(),
                                    vscore.k2(),
                                    vscore.k3(),
                                    info_k1_vec[pos_i],
                                    info_k2_vec[pos_i],
                                    info_k3_vec[pos_i],
                                    mtype,
                                    &(cur_res_appr.optim_views_[pos_cnt]),
                                    &(cur_res_appr.optim_vals_[pos_cnt]));
        time_appr_query[mtype] += timer.stop();
        timer.start();
        OptimOrientRes& cur_res_exact = res_exact[mtype];
        getOptimalViewFromExactInfo(rot_samples,
                                    pos_vec[pos_i],
                                    points,
                                    cam,
                                    mtype,
                                    &(cur_res_exact.optim_views_[pos_cnt]),
                                    &(cur_res_exact.optim_vals_[pos_cnt]));
        time_exact[mtype] += timer.stop();
      }
      pos_cnt++;
    }
  }
  CHECK_EQ(pos_cnt, n_all_pos);

  // print and save
  std::cout << "Time (sec):\n";
  std::cout << "\nType\tApprox\tExact\n";
  for (const InfoMetricType mtype : test_types)
  {
    std::cout << kInfoMetricNames[mtype] << "\t" << time_appr_query[mtype]
              << "\t" << time_exact[mtype] << std::endl;
  }
  std::cout << "The construction time for Approx is " << time_appr_constr
            << std::endl;

  for (const InfoMetricType mtype : test_types)
  {
    res_appr[mtype].save(FLAGS_abs_trace_dir);
    res_exact[mtype].save(FLAGS_abs_trace_dir);
  }
}
