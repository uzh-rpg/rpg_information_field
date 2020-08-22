#include "act_map/quadratic_factor_ops.h"
#include "act_map/positional_factor_voxel_ops.h"

#include <random>

#include <rpg_common/fs.h>
#include <rpg_common/main.h>
#include <rpg_common/timer.h>

#include <act_map/quadratic_vis_score.h>

DEFINE_int32(n_voxels, 200000, "Number of voxels to construct the kernels");
DEFINE_int32(n_points, 5000, "Number of voxels to construct the kernels");
DEFINE_int32(n_queries, 5000, "Number of voxels to construct the kernels");
DEFINE_double(n_viz_ratio, 0.1, "Ratio of visible points");

using namespace act_map;

RPG_COMMON_MAIN
{
  std::cout << "This is to test the time and memory complexity with respect "
               "to different number of voxels, points and query times.\n";

  const int n_viz = static_cast<int>(FLAGS_n_points * FLAGS_n_viz_ratio);

  std::cout << "Experiment parameters:\n"
            << "- n_voxel: " << FLAGS_n_voxels << std::endl
            << "- n_points: " << FLAGS_n_points << std::endl
            << "- n_queries: " << FLAGS_n_queries << std::endl
            << "- visible percent: " << FLAGS_n_viz_ratio * 100
            << "%, which is " << n_viz << " points.\n";

  std::srand(std::time(0));

  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<int> pt_uni(0, FLAGS_n_points - 1);
  std::uniform_int_distribution<int> vox_uni(0, FLAGS_n_voxels - 1);

  double hfov_rad = M_PI_4;
  QuadraticVisScore vscore(hfov_rad);
  vscore.initSecondOrderApprox(0.9, 0.9);

  std::string dir;
  std::string fn;
  rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
  std::string gp_vis_profile =
      dir + "/../params/fov_approximator_gp/"
            //                                     "fov50_fs50_lm1000_k10";
            //                                     "fov45_fs30_lm1000_k10";
            //                                     "fov45_fs50_lm1000_k10_fast";
            "fov45_fs30_lm1000_k10_fast";
  VLOG(1) << "Loading GP visibility profile at " << gp_vis_profile;
  VisApproxPtr<GPVisApprox> gp_vis_ptr =
      std::make_shared<GPVisibilityApproximator>();
  gp_vis_ptr->load(gp_vis_profile);
  GPInfoVoxel::setVisApprox(gp_vis_ptr);
  GPTraceVoxel::setVisApprox(gp_vis_ptr);
  VLOG(1) << *gp_vis_ptr;

  std::cout << "Generating random points...\n";
  Eigen::Matrix3Xd points_w;
  points_w.resize(Eigen::NoChange, FLAGS_n_points);
  points_w.setRandom();

  std::cout << "Generating random voxel locations...\n";
  rpg::PositionVec vox_pos(FLAGS_n_voxels);
  for (rpg::Position& p : vox_pos)
  {
    p.setRandom();
  }

  std::cout << "Generating random rotations to query...\n";
  std::vector<rpg::Rotation> query_rot(FLAGS_n_queries);
  for (rpg::Rotation& p : query_rot)
  {
    p.setRandom();
  }

  rpg::Timer timer;
  double tquery_direct_sec, tbuild_info_sec, tquery_info_sec, tbuild_trace_sec,
      tquery_trace_sec;
  double tbuild_gp_info_sec, tquery_gp_info_sec, tbuild_gp_trace_sec,
      tquery_gp_trace_sec;
  int mem_direct_byte, mem_info_byte, mem_trace_byte;
  int mem_gp_info_byte, mem_gp_trace_byte;

  {
    std::cout << ">>>> 1. Directly using point cloud.\n";
    timer.start();
    for (int q_idx = 0; q_idx < FLAGS_n_queries; q_idx++)
    {
      rpg::Matrix66 H_i;
      H_i.setZero();
      rpg::Pose Twc_i(query_rot[q_idx], vox_pos[vox_uni(rng)]);
      rpg::Pose Tcw_i = Twc_i.inverse();
      for (int viz_idx = 0; viz_idx < n_viz; viz_idx++)
      {
        Eigen::Vector3d pw = points_w.col(pt_uni(rng));
        Eigen::Vector3d pc = Tcw_i * pw;
        rpg::Matrix36 J =
            vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Tcw_i);
        H_i += J.transpose() * J * vscore.secondOrderVisibility(pc);
      }
    }
    tquery_direct_sec = timer.stop();
    mem_direct_byte = FLAGS_n_points * 3 * sizeof(double);

    std::cout << "Direct query took (sec) " << tquery_direct_sec << std::endl;
    std::cout << "Memory cost (byte KB MB): " << mem_direct_byte << ", "
              << mem_direct_byte / 1024.0 << ", "
              << mem_direct_byte / (1024.0 * 1024.0) << std::endl;
  }

  {
    std::cout << ">>>> 2 Construct Info Kernel.\n";
    timer.start();
    InfoK1Vec info_k1_vec(0);
    InfoK2Vec info_k2_vec(0);
    InfoK3Vec info_k3_vec(0);
    double cnt_r = 0.0;
    for (int vox_idx = 0; vox_idx < FLAGS_n_voxels; vox_idx++)
    {
      if (vox_idx > FLAGS_n_voxels * cnt_r)
      {
        std::cout << "." << std::flush;
        cnt_r += 0.1;
      }
      info_k1_vec.emplace_back(InfoK1());
      info_k2_vec.emplace_back(InfoK2());
      info_k3_vec.emplace_back(InfoK3());
      Eigen::Vector3d twc = vox_pos[vox_idx];
      for (int viz_idx = 0; viz_idx < n_viz; viz_idx++)
      {
        Eigen::Vector3d pw = points_w.col(pt_uni(rng));
        assignToFactor(pw, twc, &(info_k1_vec[vox_idx]),
                       &(info_k2_vec[vox_idx]), &(info_k3_vec[vox_idx]));
      }
    }
    tbuild_info_sec = timer.stop();
    std::cout << "Building info kernels took (sec) " << tbuild_info_sec
              << std::endl;

    timer.start();
    for (int q_idx = 0; q_idx < FLAGS_n_queries; q_idx++)
    {
      rpg::Matrix66 H_i;
      int rvox_idx = vox_uni(rng);
      Eigen::Matrix3d Rwc = query_rot[q_idx].getRotationMatrix();
      getInfoAtRotation(Rwc, vscore.k1(), vscore.k2(), vscore.k3(),
                        info_k1_vec[rvox_idx], info_k2_vec[rvox_idx],
                        info_k3_vec[rvox_idx], &H_i);
    }
    tquery_info_sec = timer.stop();
    mem_info_byte = info_k1_vec.size() * sizeof(InfoK1) +
                    info_k2_vec.size() * sizeof(InfoK2) +
                    info_k3_vec.size() * sizeof(InfoK3);

    std::cout << "Query using info kernels took (sec) " << tquery_info_sec
              << std::endl;
    std::cout << "Memory cost (byte, KB, MB): " << mem_info_byte << ", "
              << mem_info_byte / 1024.0 << ", "
              << mem_info_byte / (1024.0 * 1024.0) << std::endl;
  }

  {
    std::cout << ">>>> 3 Construct Trace Kernel.\n";
    timer.start();
    TraceK1Vec trace_k1_vec(0);
    TraceK2Vec trace_k2_vec(0);
    TraceK3Vec trace_k3_vec(0);
    double cnt_r = 0.0;
    for (int vox_idx = 0; vox_idx < FLAGS_n_voxels; vox_idx++)
    {
      if (vox_idx > FLAGS_n_voxels * cnt_r)
      {
        std::cout << "." << std::flush;
        cnt_r += 0.1;
      }
      trace_k1_vec.emplace_back(TraceK1());
      trace_k2_vec.emplace_back(TraceK2());
      trace_k3_vec.emplace_back(TraceK3());
      Eigen::Vector3d twc = vox_pos[vox_idx];
      for (int viz_idx = 0; viz_idx < n_viz; viz_idx++)
      {
        Eigen::Vector3d pw = points_w.col(pt_uni(rng));
        assignToFactor(pw, twc, &(trace_k1_vec[vox_idx]),
                       &(trace_k2_vec[vox_idx]), &(trace_k3_vec[vox_idx]));
      }
    }
    tbuild_trace_sec = timer.stop();
    std::cout << "Building trace kernels took (sec) " << tbuild_trace_sec
              << std::endl;

    timer.start();
    for (int q_idx = 0; q_idx < FLAGS_n_queries; q_idx++)
    {
      int rvox_idx = vox_uni(rng);
      Eigen::Matrix3d Rwc = query_rot[q_idx].getRotationMatrix();
      getTraceAtRotation(Rwc, vscore.k1(), vscore.k2(), vscore.k3(),
                         trace_k1_vec[rvox_idx], trace_k2_vec[rvox_idx],
                         trace_k3_vec[rvox_idx]);
    }
    tquery_trace_sec = timer.stop();
    mem_trace_byte = trace_k1_vec.size() * sizeof(TraceK1) +
                     trace_k2_vec.size() * sizeof(TraceK2) +
                     trace_k3_vec.size() * sizeof(TraceK3);
    std::cout << "Query using trace kernels took (sec) " << tquery_trace_sec
              << std::endl;
    std::cout << "Memory cost (byte, KB, MB) : " << mem_trace_byte << ", "
              << mem_trace_byte / 1024.0 << ", "
              << mem_trace_byte / (1024.0 * 1024.0) << std::endl;
  }

  {
    std::cout << ">>>> 4 Construct GP Info Kernel.\n";
    timer.start();
    rpg::Timer local_timer;
    local_timer.start();
    GPInfoVoxelVec gp_info_vec(FLAGS_n_voxels);
    VLOG(1) << " - allocation: " << local_timer.stop();
    double cnt_r = 0.0;

    local_timer.start();
    for (int vox_idx = 0; vox_idx < FLAGS_n_voxels; vox_idx++)
    {
      if (vox_idx > FLAGS_n_voxels * cnt_r)
      {
        std::cout << "." << std::flush;
        cnt_r += 0.1;
      }

      const Eigen::Vector3d twc = vox_pos[vox_idx];
      for (int viz_idx = 0; viz_idx < n_viz; viz_idx++)
      {
        Eigen::Vector3d pw = points_w.col(pt_uni(rng));
        addToFactorVoxel(pw, twc, &(gp_info_vec[vox_idx]));
      }
    }
    VLOG(1) << "- actual compute: " << local_timer.stop();

    tbuild_gp_info_sec = timer.stop();
    std::cout << "Building GP info kernels took (sec) " << tbuild_gp_info_sec
              << std::endl;
    timer.start();
    for (int q_idx = 0; q_idx < FLAGS_n_queries; q_idx++)
    {
      int rvox_idx = vox_uni(rng);
      Eigen::Matrix3d Rwc = query_rot[q_idx].getRotationMatrix();
      GPInfoVoxel::OutMatType out;
      gp_info_vec[rvox_idx].queryAtRotation(Rwc, &out);
    }
    tquery_gp_info_sec = timer.stop();
    mem_gp_info_byte = gp_info_vec.size() * sizeof(double) *
                       GPInfoVoxel::kNumOut * GPInfoVoxel::kNumOut *
                       GPInfoVoxel::dimFeatures();
    std::cout << "Query using GP info kernels took (sec) " << tquery_gp_info_sec
              << std::endl;
    std::cout << "Memory cost (byte, KB, MB) : " << mem_gp_info_byte << ", "
              << mem_gp_info_byte / 1024.0 << ", "
              << mem_gp_info_byte / (1024.0 * 1024.0) << std::endl;
    //    GPInfoVoxel::printTiming();
    //    gp_vis_ptr->impl()->printTiming();
    //    gp_vis_ptr->impl()->resetTimer();
  }

  {
    std::cout << ">>>> 5 Construct GP Trace Kernel.\n";
    timer.start();
    GPTraceVoxelVec gp_trace_vec(FLAGS_n_voxels);
    double cnt_r = 0.0;
    for (int vox_idx = 0; vox_idx < FLAGS_n_voxels; vox_idx++)
    {
      if (vox_idx > FLAGS_n_voxels * cnt_r)
      {
        std::cout << "." << std::flush;
        cnt_r += 0.1;
      }

      const Eigen::Vector3d twc = vox_pos[vox_idx];
      for (int viz_idx = 0; viz_idx < n_viz; viz_idx++)
      {
        Eigen::Vector3d pw = points_w.col(pt_uni(rng));
        addToFactorVoxel(pw, twc, &(gp_trace_vec[vox_idx]));
      }
    }
    tbuild_gp_trace_sec = timer.stop();
    std::cout << "Building GP trace kernels took (sec) " << tbuild_gp_trace_sec
              << std::endl;
    timer.start();
    for (int q_idx = 0; q_idx < FLAGS_n_queries; q_idx++)
    {
      int rvox_idx = vox_uni(rng);
      Eigen::Matrix3d Rwc = query_rot[q_idx].getRotationMatrix();
      GPTraceVoxel::OutMatType out;
      gp_trace_vec[rvox_idx].queryAtRotation(Rwc, &out);
    }
    tquery_gp_trace_sec = timer.stop();
    mem_gp_trace_byte = gp_trace_vec.size() * sizeof(double) *
                        GPTraceVoxel::kNumOut * GPTraceVoxel::kNumOut *
                        GPTraceVoxel::dimFeatures();
    std::cout << "Query using trace kernels took (sec) " << tquery_gp_trace_sec
              << std::endl;
    std::cout << "Memory cost (byte, KB, MB) : " << mem_gp_trace_byte << ", "
              << mem_gp_trace_byte / 1024.0 << ", "
              << mem_gp_trace_byte / (1024.0 * 1024.0) << std::endl;
    //    GPTraceVoxel::printTiming();
    //    gp_vis_ptr->impl()->printTiming();
  }

  return 0;
}
