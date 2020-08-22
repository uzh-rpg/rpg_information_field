#include "act_map/act_map.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

#include <rpg_common/main.h>
#include <rpg_common/timer.h>
#include <vi_utils/map.h>
#include <vi_utils/common_utils.h>

#include "act_map/sampler.h"

// points to calculate
// we could use a predefined map
DEFINE_string(abs_map, "", "absolute path to the map file.");
// or random points within some ranges
DEFINE_int32(n_points, 1000, "Number of points in the world");
DEFINE_double(xrange, 10.0, "X range centered at zero.");
DEFINE_double(yrange, 10.0, "Y range centered at zero.");
DEFINE_double(zrange, 5.0, "Z range centered at zero.");
DEFINE_double(pos_factor_layer_margin, 1.0, "derive ranges to allocate kernel layer.");

// camera settings
DEFINE_string(gp_nm, "fov45_fs30_lm1000_k10_fast", "gp folder under params/fov_approximator_gp");
DEFINE_double(hfov_deg, 45.0, "half fov for quadratic approximation");
DEFINE_string(cam_nm, "hfov45", "camera folder under params/cameras");

// gp visibilities to test
DEFINE_string(gp_list, "", "list of GP visibility approximation to test.");

// layer settings
DEFINE_double(occ_vox_size, 0.01, "voxel size of the occupancy layer");
DEFINE_double(ker_vox_size, 0.5, "voxel size of the kernel layer");
DEFINE_double(ker_nvox_per_side, 8, "voxel size of the kernel layer");

// how many queries to test
DEFINE_int32(n_optim_orient, 200, "number of sampled positions for comparing optim orientations");
DEFINE_int32(n_query, 200, "number of sampled positions for comparing FIM");
DEFINE_int32(n_cont_trans_sample, 200, "Number of samples for continuous translation.");
DEFINE_int32(cont_rot_sample_deg, 5.0, "Intervals for sampling continuous rotations.");

using namespace act_map;

namespace
{
constexpr InfoMetricType kContType = InfoMetricType::kDet;

void readGPListAndNames(const std::string& abs_list, std::vector<std::string>* gp_dirs,
                        std::vector<std::string>* gp_labels)
{
  std::ifstream ifs(abs_list.c_str(), std::ios::in);
  CHECK(ifs.is_open());
  std::string line;
  while (std::getline(ifs, line))
  {
    std::string nm_i, label_i;
    std::stringstream s_stream(line);

    s_stream >> nm_i >> label_i;
    gp_dirs->push_back(nm_i);
    gp_labels->push_back(label_i);
  }
}

std::ofstream& saveVec3dVec(const Vec3dVec& data, std::ofstream& ofs,
                            const std::string sep = std::string(" "))
{
  for (size_t i = 0; i < data.size(); i++)
  {
    const Eigen::Vector3d& v = data[i];
    ofs << v.x() << sep << v.y() << sep << v.z() << std::endl;
  }

  return ofs;
}

std::ofstream& saveVector(const std::vector<double>& data, std::ofstream& ofs,
                          const std::string& sep = std::string(" "))
{
  for (size_t i = 0; i < data.size(); i++)
  {
    ofs << data[i] << sep;
  }
  ofs << std::endl;
  return ofs;
}

std::ofstream& saveInfoVec(const InfoVec& info_vec, std::ofstream& ofs,
                           const std::string sep = std::string(" "))
{
  for (size_t i = 0; i < info_vec.size(); i++)
  {
    const Info& fim = info_vec[i];
    for (int ri = 0; ri < 6; ri++)
    {
      for (int ci = 0; ci < 6; ci++)
      {
        ofs << fim(ri, ci) << sep;
      }
    }
    ofs << std::endl;
  }
  return ofs;
}

struct ExpRes
{
  // general info.
  int n_ker_voxel = -1;
  double time_construct = -1.0;
  double ker_mem_kb = -1.0;
  double pc_mem_kb = -1.0;

  // query time
  std::unordered_map<InfoMetricType, std::vector<double>> query_t_map;
  std::unordered_map<InfoMetricType, std::vector<double>> query_t_pc;

  // optim orientations
  Vec3dVec optim_orient_vox_centers;
  std::unordered_map<InfoMetricType, Vec3dVec> optim_orient_map;
  std::unordered_map<InfoMetricType, Vec3dVec> optim_orient_pc;

  // fim
  Vec3dVec fim_vox_centers;
  InfoVec fims_map;
  InfoVec fims_pc;
  std::vector<double> fim_time_map;
  std::vector<double> fim_time_pc;

  // continuous motion
  std::vector<double> metric_continuous_rot_map;
  std::vector<double> metric_continuous_rot_pc;
  std::vector<double> metric_continuous_trans_map;
  std::vector<double> metric_continuous_trans_pc;
};

void saveExpRes(const ExpRes& res, const std::string& save_dir, const std::string& nm)
{
  const std::string sep(" ");

  {
    const std::string gen_fn = save_dir + "/general_info_" + nm + ".txt";
    std::ofstream gen_ofs(gen_fn);
    gen_ofs << "# general info.: n_ker_vox t_construct ker_mem_kb pc_mem_kb\n";
    gen_ofs << res.n_ker_voxel << sep << res.time_construct << sep << res.ker_mem_kb << sep
            << res.pc_mem_kb << std::endl;
  }

  if (res.fim_vox_centers.size() > 0)
  {
    const std::string fim_vox_c_fn = save_dir + "/fim_vox_c_" + nm + ".txt";
    std::ofstream fim_vox_c_fs(fim_vox_c_fn);
    fim_vox_c_fs << "# each row is the corresponding voxel center.\n";
    saveVec3dVec(res.fim_vox_centers, fim_vox_c_fs);
    {
      const std::string fim_pc_fn = save_dir + "/fim_pc_" + nm + ".txt";
      std::ofstream fims_pc_ofs(fim_pc_fn);
      fims_pc_ofs << "# each row is a flattened 6x6 matrix\n";
      saveInfoVec(res.fims_pc, fims_pc_ofs);

      const std::string fim_time_pc_fn = save_dir + "/fim_time_pc_" + nm + ".txt";
      std::ofstream fim_time_pc_ofs(fim_time_pc_fn);
      fim_time_pc_ofs << "# each item is the time to get the FIM\n";
      saveVector(res.fim_time_pc, fim_time_pc_ofs);
    }
    {
      const std::string fim_map_fn = save_dir + "/fim_map_" + nm + ".txt";
      std::ofstream fims_map_ofs(fim_map_fn);
      fims_map_ofs << "# each row is a flattened 6x6 matrix\n";
      saveInfoVec(res.fims_map, fims_map_ofs);

      const std::string fim_time_map_fn = save_dir + "/fim_time_map_" + nm + ".txt";
      std::ofstream fim_time_map_ofs(fim_time_map_fn);
      fim_time_map_ofs << "# each item is the time to get the FIM\n";
      saveVector(res.fim_time_map, fim_time_map_ofs);
    }
  }

  {
    const std::string optim_orient_vox_c_fn = save_dir + "/optim_orient_vox_c_" + nm + ".txt";
    std::ofstream optim_orient_vox_c_fs(optim_orient_vox_c_fn);
    optim_orient_vox_c_fs << "# each row is the corresponding voxel center.\n";
    saveVec3dVec(res.optim_orient_vox_centers, optim_orient_vox_c_fs);
    optim_orient_vox_c_fs.close();

    // each info. metric
    for (const auto& p : res.optim_orient_map)
    {
      const InfoMetricType type = p.first;
      CHECK(res.optim_orient_pc.find(type) != res.optim_orient_pc.end());
      const std::string info_t_nm = kInfoMetricNames[type];
      CHECK_EQ(res.optim_orient_map.at(type).size(), res.optim_orient_pc.at(type).size());

      {
        const std::string optim_orient_map_fn =
            save_dir + "/optim_orient_map_" + info_t_nm + "_" + nm + ".txt";
        std::ofstream optim_orient_map_fs(optim_orient_map_fn);
        optim_orient_map_fs << "# each row is an optimal orientation\n";
        saveVec3dVec(res.optim_orient_map.at(type), optim_orient_map_fs);
      }
      {
        const std::string optim_orient_pc_fn =
            save_dir + "/optim_orient_pc_" + info_t_nm + "_" + nm + ".txt";
        std::ofstream optim_orient_pc_fs(optim_orient_pc_fn);
        optim_orient_pc_fs << "# each row is an optimal orientation\n";
        saveVec3dVec(res.optim_orient_pc.at(type), optim_orient_pc_fs);
      }
    }

    if (!res.metric_continuous_rot_map.empty())
    {
      const size_t n_cont_rot = res.metric_continuous_rot_map.size();
      CHECK_EQ(n_cont_rot, res.metric_continuous_rot_pc.size());
      const size_t n_cont_trans = res.metric_continuous_trans_map.size();
      CHECK_EQ(n_cont_trans, res.metric_continuous_trans_pc.size());

      {
        const std::string cont_rot_fn = save_dir + "/metric_cont_rot_" + nm + ".txt";
        std::ofstream cont_rot_fs(cont_rot_fn);
        cont_rot_fs << "# first row: from map; second row: from pc\n";
        saveVector(res.metric_continuous_rot_map, cont_rot_fs);
        saveVector(res.metric_continuous_rot_pc, cont_rot_fs);
      }

      {
        const std::string cont_trans_fn = save_dir + "/metric_cont_trans_" + nm + ".txt";
        std::ofstream cont_trans_fs(cont_trans_fn);
        cont_trans_fs << "# first row: from map; second row: from pc\n";
        saveVector(res.metric_continuous_trans_map, cont_trans_fs);
        saveVector(res.metric_continuous_trans_pc, cont_trans_fs);
      }
    }
  }

  {
    for (const auto& p : res.query_t_map)
    {
      const InfoMetricType& type = p.first;
      CHECK(res.optim_orient_pc.find(type) != res.optim_orient_pc.end());
      const std::string info_t_nm = kInfoMetricNames[type];
      {
        const std::string t_query_map_fn =
            save_dir + "/t_query_map_" + info_t_nm + "_" + nm + ".txt";
        std::ofstream t_query_map_fs(t_query_map_fn);
        t_query_map_fs << "# each number is one query time in second.\n";
        saveVector(res.query_t_map.at(type), t_query_map_fs);
      }

      {
        const std::string t_query_pc_fn = save_dir + "/t_query_pc_" + info_t_nm + "_" + nm + ".txt";
        std::ofstream t_query_pc_fs(t_query_pc_fn);
        t_query_pc_fs << "# each number is one query time in second.\n";
        saveVector(res.query_t_pc.at(type), t_query_pc_fs);
      }
    }
  }
}

template <typename VoxelT>
void testMap(const ActMapOptions& options, const Eigen::Matrix3Xd& pts,
             const Vec3dVec& optim_orient_query_pos, const rpg::PoseVec& query_poses,
             const rpg::PoseVec& cont_rot_poses, const rpg::PoseVec& cont_trans_poses,
             const std::vector<double> pos_factor_layer_ranges, ExpRes* res)
{
  ActMap<VoxelT> act_map(options);
  const InfoMetricTypeVec supported_info_metrics = act_map.supportedInfoMetricTypes();

  std::cout << "- setting world points.\n";
  act_map.setOccupancyWorldPoints(pts);
  std::cout << "- allocating kernel layer.\n";
  act_map.allocateFactorLayerUniform(pos_factor_layer_ranges);
  res->n_ker_voxel = act_map.kerLayerCRef().getNumberOfAllocatedVoxels();
  res->ker_mem_kb = act_map.posFactorVoxelNumCoefs() * res->n_ker_voxel * sizeof(double) / 1024.0;
  std::cout << "allocated " << res->n_ker_voxel << " kernel voxels, " << res->ker_mem_kb << " kb."
            << std::endl;
  std::cout << "- computing kernel layer.\n";
  rpg::Timer timer;
  timer.start();
  act_map.recomputeFactorLayer();
  res->time_construct = timer.stop();
  std::cout << "- preparing info. from pointclouds.\n";
  act_map.prepareInfoFromPointCloud();
  res->pc_mem_kb = act_map.cachedPoints().size() * 3 * sizeof(double) / 1024.0;

  size_t n_optim_orient = optim_orient_query_pos.size();
  // optim orientations
  std::cout << "- compute optim orientations from map\n";
  for (const InfoMetricType type : supported_info_metrics)
  {
    // from both map and pointcloud
    res->optim_orient_map.insert(std::pair<InfoMetricType, Vec3dVec>(type, {}));

    Eigen::Vector3d vox_c, best_view_map;
    for (const Eigen::Vector3d& pos : optim_orient_query_pos)
    {
      act_map.getBestViewFromNearestVoxel(pos, type, &vox_c, &best_view_map);
      if (res->optim_orient_vox_centers.size() != n_optim_orient)
      {
        res->optim_orient_vox_centers.push_back(vox_c);
      }
      res->optim_orient_map[type].push_back(best_view_map);
    }
    CHECK_EQ(res->optim_orient_vox_centers.size(), res->optim_orient_map[type].size());
  }

  std::cout << "- compute optim orientations from pc\n";
  for (const InfoMetricType type : supported_info_metrics)
  {
    Eigen::Vector3d best_view_pc;
    res->optim_orient_pc.insert(std::pair<InfoMetricType, Vec3dVec>(type, {}));
    for (const Eigen::Vector3d& vox_c : res->optim_orient_vox_centers)
    {
      act_map.getBestViewFromPCNoVisCheck(vox_c, type, &best_view_pc);
      res->optim_orient_pc[type].push_back(best_view_pc);
    }
  }

  double info_m_val;
  std::cout << "- timing queries from map\n";
  for (const InfoMetricType type : supported_info_metrics)
  {
    res->query_t_map.insert(std::pair<InfoMetricType, std::vector<double>>(type, {}));
    for (const rpg::Pose& Twc : query_poses)
    {
      timer.start();
      act_map.getInfoMetricAt(Twc, type, &info_m_val);
      res->query_t_map[type].push_back(timer.stop());
    }
  }
  std::cout << "- timing queries from PC\n";
  for (const InfoMetricType type : supported_info_metrics)
  {
    res->query_t_pc.insert(std::pair<InfoMetricType, std::vector<double>>(type, {}));
    for (const rpg::Pose& Twc : query_poses)
    {
      timer.start();
      act_map.getInfoMetricFromPC(Twc, type, &info_m_val);
      res->query_t_pc[type].push_back(timer.stop());
    }
  }

  if (act_map.supportFullFIM())
  {
    std::cout << "- compute FIM from map\n";
    Info fim;
    Eigen::Vector3d vox_c;
    size_t n_query = query_poses.size();

    for (const rpg::Pose& Twc : query_poses)
    {
      timer.start();
      act_map.getFIMFromNearestVoxel(Twc, &vox_c, &fim);
      res->fim_time_map.push_back(timer.stop());
      res->fim_vox_centers.push_back(vox_c);
      res->fims_map.push_back(fim);
    }
    CHECK_EQ(res->fim_vox_centers.size(), res->fims_map.size());
    std::cout << "- compute FIM from pc\n";
    for (size_t i = 0; i < n_query; i++)
    {
      rpg::Pose Twc = query_poses[i];
      Twc.getPosition() = res->fim_vox_centers[i];
      timer.start();
      act_map.getFIMFromPC(Twc, &fim);
      res->fim_time_pc.push_back(timer.stop());
      res->fims_pc.push_back(fim);
    }
  }
  else
  {
    std::cout << "- map does not support FIM, not computing.\n";
  }

  if (std::find(supported_info_metrics.begin(), supported_info_metrics.end(), kContType) !=
      supported_info_metrics.end())
  {
    std::cout << "- compute metrics for continuous motion.\n";
    for (const rpg::Pose& Twc : cont_rot_poses)
    {
      double val;
      act_map.getInfoMetricAt(Twc, kContType, &val);
      res->metric_continuous_rot_map.push_back(val);
      act_map.getInfoMetricFromPC(Twc, kContType, &val);
      res->metric_continuous_rot_pc.push_back(val);
    }

    for (const rpg::Pose& Twc : cont_trans_poses)
    {
      double val;
      act_map.getInfoMetricAt(Twc, kContType, &val);
      res->metric_continuous_trans_map.push_back(val);
      act_map.getInfoMetricFromPC(Twc, kContType, &val);
      res->metric_continuous_trans_pc.push_back(val);
    }
  }
  else
  {
    std::cout << "- not computing continuous motion for this map\n";
  }
}

}  // namespace

RPG_COMMON_MAIN
{
  std::cout << "This is to compute and save the results from different maps for comparison.\n";

  std::cout << "Experiment parameters:\n"
            << "- n_points: " << FLAGS_n_points << std::endl
            << "- abs_map: " << FLAGS_abs_map << std::endl
            << "- xrange: " << FLAGS_xrange << std::endl
            << "- yrange: " << FLAGS_yrange << std::endl
            << "- zrange: " << FLAGS_zrange << std::endl
            << "- ker. layer range margin: " << FLAGS_pos_factor_layer_margin << std::endl
            << "- gp_nm: " << FLAGS_gp_nm << std::endl
            << "- hfov_deg: " << FLAGS_hfov_deg << std::endl
            << "- cam_nm: " << FLAGS_cam_nm << std::endl
            << "- gp_list: " << FLAGS_gp_list << std::endl
            << "- occ. layer vox size: " << FLAGS_occ_vox_size << std::endl
            << "- ker. layer vox size: " << FLAGS_ker_vox_size << std::endl
            << "- ker. n. vox per side: " << FLAGS_ker_nvox_per_side << std::endl
            << "- n optim orientations: " << FLAGS_n_optim_orient << std::endl
            << "- n FIM: " << FLAGS_n_query << std::endl
            << "- n cont. trans. samples: " << FLAGS_n_cont_trans_sample << std::endl
            << "- n cont. rot. sample deg.: " << FLAGS_cont_rot_sample_deg << std::endl;

  std::string dir;
  std::string fn;
  rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);

  std::vector<std::string> gp_names;
  std::vector<std::string> gp_labels;
  std::vector<double> quadratic_boundary_vals{ 0.5, 0.8 };
  std::vector<std::string> quadratic_suffix{ std::string("_0p5"), std::string("_0p8") };

  if (FLAGS_gp_list.empty())
  {
    gp_names.push_back(FLAGS_gp_nm);
    gp_labels.push_back(std::string(""));
  }
  else
  {
    std::string gp_list_fn = dir + "/../params/" + FLAGS_gp_list;
    CHECK(rpg::fs::fileExists(gp_list_fn));
    std::cout << "GP approx. provided, reading from " << gp_list_fn << std::endl;
    readGPListAndNames(gp_list_fn, &gp_names, &gp_labels);
    CHECK_GT(gp_names.size(), 0u);
    CHECK_EQ(gp_names.size(), gp_labels.size());
    std::cout << "Found the following to test:\n";
    for (size_t i = 0; i < gp_names.size(); i++)
    {
      std::cout << "- " << gp_names[i] << ", " << gp_labels[i] << std::endl;
    }
  }
  std::cout << "Have " << gp_names.size() << " GP visibility approx. to test.\n";

  std::cout << "\n===== 0. check all directories are there ====\n";

  std::vector<std::string> gp_dirs;
  for (const std::string& v : gp_names)
  {
    std::string gp_dir = dir + "/../params/fov_approximator_gp/" + v;
    gp_dirs.push_back(gp_dir);
    CHECK(rpg::fs::pathExists(gp_dir));
    std::cout << "- found gp dir: " << gp_dir << std::endl;
  }

  std::string cam_dir = dir + "/../params/cameras/" + FLAGS_cam_nm;
  CHECK(rpg::fs::pathExists(cam_dir));
  std::cout << "- camera dir: " << cam_dir << std::endl;

  std::string top_trace_dir = dir + "/../trace/exp_compare_diff_maps/";
  std::cout << "- top trace dir: " << top_trace_dir << std::endl;
  if (!rpg::fs::pathExists(top_trace_dir))
  {
    std::string mkdir_cmd = "mkdir -p " + top_trace_dir;
    system(mkdir_cmd.c_str());
  }
  CHECK(rpg::fs::pathExists(top_trace_dir));

  std::cout << "\n===== 1. loading/generating points =====\n" << std::endl;
  Eigen::Matrix<double, 3, Eigen::Dynamic> pts;
  if (FLAGS_abs_map.empty())
  {
    std::cout << "Map not provided, will generate random points." << std::endl;
    utils::generateRandomPointsWithin(FLAGS_n_points, FLAGS_xrange, FLAGS_yrange, FLAGS_zrange,
                                      &pts);
  }
  else
  {
    std::cout << "Loading from " << FLAGS_abs_map;
    vi_utils::Map map;
    map.load(FLAGS_abs_map, std::string());
    pts = map.points_;
  }
  std::cout << "We have " << pts.cols() << " points in the world." << std::endl;

  std::cout << "\n==== 2. loading/setting cameras =====\n" << std::endl;
  {
    vi::PinholeCamPtr cam_ptr = vi::PinholeCam::loadFromDir(cam_dir);
    std::cout << *cam_ptr;
  }
  ActMapOptions options;
  options.vis_checker_options_.use_camera_ = true;
  options.vis_checker_options_.cam_dir_ = cam_dir;
  options.vis_options_.resize(1);
  options.vis_options_[0].half_fov_rad = FLAGS_hfov_deg * 1.0 / 180.0 * M_PI;
  options.occ_layer_options_.vox_size = FLAGS_occ_vox_size;
  options.pos_factor_layer_options_.vox_size = FLAGS_ker_vox_size;
  options.pos_factor_layer_options_.vox_per_side = FLAGS_ker_nvox_per_side;
  options.vis_checker_options_.use_view_filtering = false;
  std::vector<double> kernel_layer_ranges{ FLAGS_xrange - FLAGS_pos_factor_layer_margin,
                                           FLAGS_yrange - FLAGS_pos_factor_layer_margin,
                                           FLAGS_zrange - FLAGS_pos_factor_layer_margin };
  kInfoMetricUseLogDet = false;
  std::vector<double> sample_position_ranges;
  for (const double v : kernel_layer_ranges)
  {
    sample_position_ranges.push_back(v * 0.8);
  }

  std::cout << "\n==== 3. different map types =====\n";

  Vec3dVec optim_orient_query_pos;
  utils::generateRandomPointsWithin(FLAGS_n_optim_orient, sample_position_ranges[0],
                                    sample_position_ranges[1], sample_position_ranges[2],
                                    &optim_orient_query_pos);

  Vec3dVec fim_pos;
  utils::generateRandomPointsWithin(FLAGS_n_query, sample_position_ranges[0],
                                    sample_position_ranges[1], sample_position_ranges[2], &fim_pos);
  rpg::PoseVec query_poses(FLAGS_n_query);
  for (int i = 0; i < FLAGS_n_query; i++)
  {
    query_poses[i].getPosition() = fim_pos[i];
    query_poses[i].getRotation().setRandom();
  }

  rpg::Rotation cam_horizontal_rot = rpg::Rotation(Eigen::Vector3d(-0.5 * M_PI, 0, 0));

  rpg::PoseVec continous_rot_poses;
  rpg::Pose cont_rot_start = rpg::Pose();
  cont_rot_start.getPosition().setZero();
  cont_rot_start.getRotation() = cam_horizontal_rot;
  const Eigen::Vector3d rot_ax(0.0, 1.0, 0.0);
  for (double rot_deg = 0; rot_deg < 360.0; rot_deg += FLAGS_cont_rot_sample_deg)
  {
    rpg::Rotation dR(Eigen::Vector3d(0, rot_deg * M_PI / 180.0, 0.0));
    rpg::Pose Ti = cont_rot_start;
    Ti.getRotation() = Ti.getRotation() * dR;
    continous_rot_poses.push_back(Ti);
  }
  std::cout << "- sampled " << continous_rot_poses.size() << " poses with continuous rotations.\n";

  rpg::PoseVec continous_trans_poses;
  std::vector<double> cont_trans_x, cont_trans_y, cont_trans_z;
  vi_utils::linspace(-0.4 * sample_position_ranges[0], 0.4 * sample_position_ranges[0],
                     static_cast<size_t>(FLAGS_n_cont_trans_sample), &cont_trans_x);
  vi_utils::linspace(-0.4 * sample_position_ranges[1], 0.4 * sample_position_ranges[1],
                     static_cast<size_t>(FLAGS_n_cont_trans_sample), &cont_trans_y);
  vi_utils::linspace(-0.4 * sample_position_ranges[2], 0.4 * sample_position_ranges[2],
                     static_cast<size_t>(FLAGS_n_cont_trans_sample), &cont_trans_z);
  CHECK_EQ(cont_trans_x.size(), cont_trans_y.size());
  CHECK_EQ(cont_trans_x.size(), cont_trans_z.size());
  for (size_t i = 0; i < cont_trans_x.size(); i++)
  {
    rpg::Pose Ti;
    Ti.getRotation() = cam_horizontal_rot;
    Ti.getPosition() = Eigen::Vector3d(cont_trans_x[i], cont_trans_y[i], cont_trans_z[i]);
    continous_trans_poses.push_back(Ti);
  }
  std::cout << "- sampled " << continous_trans_poses.size()
            << " poses with continuous positions.\n";

  for (size_t qi = 0; qi < quadratic_boundary_vals.size(); qi++)
  {
    const double& qval = quadratic_boundary_vals[qi];
    options.vis_options_[0].boundary_to_mid_ratio = qval;
    options.vis_options_[0].boundary_value = qval;
    std::cout << "\n============================\n";
    std::cout << "\n===> quadratic info. map  "<< qval << std::endl;
    std::cout << "\n============================\n";
    {
      ExpRes quad_info_res;
      testMap<QuadInfoVoxel>(options, pts, optim_orient_query_pos, query_poses, continous_rot_poses,
                        continous_trans_poses, kernel_layer_ranges, &quad_info_res);
      saveExpRes(quad_info_res, top_trace_dir, "quad_info" + quadratic_suffix[qi]);
    }

    std::cout << "\n============================\n";
    std::cout << "\n===> quadratic trace map " << qval << std::endl;
    std::cout << "\n============================\n";
    {
      ExpRes quad_trace_res;
      testMap<QuadTraceVoxel>(options, pts, optim_orient_query_pos, query_poses, continous_rot_poses,
                          continous_trans_poses, kernel_layer_ranges, &quad_trace_res);
      saveExpRes(quad_trace_res, top_trace_dir, "quad_trace" + quadratic_suffix[qi]);
    }
  }

  std::cout << "\n============================\n";
  std::cout << "\n===> test for different visibility approximations\n";
  std::cout << "\n============================\n";
  for (size_t i = 0; i < gp_dirs.size(); i++)
  {
    std::cout << "\n===> Visibility approximation " << gp_dirs[i] << " (" << i + 1 << " out of "
              << gp_dirs.size() << ") "
              << "\n";
    const std::string& label_i = gp_labels[i];
    GPVisApproxPtr gp_vis_ptr = std::make_shared<GPVisibilityApproximator>();
    gp_vis_ptr->load(gp_dirs[i]);
    GPInfoVoxel::setVisApprox(gp_vis_ptr);
    GPTraceVoxel::setVisApprox(gp_vis_ptr);

    std::cout << "\n============================\n";
    std::cout << "\n===> GP info. map\n";
    std::cout << "\n============================\n";
    {
      ExpRes gp_info_res;
      testMap<GPInfoVoxel>(options, pts, optim_orient_query_pos, query_poses, continous_rot_poses,
                           continous_trans_poses, kernel_layer_ranges, &gp_info_res);
      saveExpRes(gp_info_res, top_trace_dir, "gp_info-" + label_i);
    }

    std::cout << "\n============================\n";
    std::cout << "\n===> GP trace map\n";
    std::cout << "\n============================\n";
    {
      ExpRes gp_trace_res;
      testMap<GPTraceVoxel>(options, pts, optim_orient_query_pos, query_poses, continous_rot_poses,
                            continous_trans_poses, kernel_layer_ranges, &gp_trace_res);
      saveExpRes(gp_trace_res, top_trace_dir, "gp_trace-" + label_i);
    }
  }

  return 0;
}
