#include "act_map_exp/quad_rrt.h"

#include <rpg_common/save.h>

namespace act_map_exp
{
const std::unordered_map<std::string, RRTCostType> kStrToRRTCostT = {
  { std::string("Dist"), RRTCostType::kDist },
  { std::string("Yaw"), RRTCostType::kYaw },
  { std::string("ESDF"), RRTCostType::kESDF },
  { std::string("Info"), RRTCostType::kInfo }
};

const std::unordered_map<RRTCostType, std::string> kRRTCostTToStr = {
  { RRTCostType::kDist, std::string("Dist") },
  { RRTCostType::kYaw, std::string("Yaw") },
  { RRTCostType::kESDF, std::string("ESDF") },
  { RRTCostType::kInfo, std::string("Info") }
};

void RRTStats::save(const std::string& dir) const
{
  Eigen::Matrix<double, Eigen::Dynamic, 5> data;
  data.resize(size(), Eigen::NoChange);

  for (int i = 0; i < size(); i++)
  {
    const size_t idx_i = static_cast<size_t>(i);

    data(i, 0) = static_cast<double>(i);
    data(i, 1) = static_cast<double>(n_iters_[idx_i]);
    data(i, 2) = best_costs_[idx_i];
    data(i, 3) = static_cast<double>(n_verts_[idx_i]);
    data(i, 4) = static_cast<double>(n_edges_[idx_i]);
    rpg::save(dir + "/rrt_verts_iter_" + std::to_string(i) + ".txt",
              vertices_[idx_i], rpg::EigenSpaceSeparatedFmt, "edges: x y z");
    rpg::save(dir + "/rrt_edges_iter_" + std::to_string(i) + ".txt",
              edge_pairs_[idx_i], rpg::EigenSpaceSeparatedFmt,
              "pair: id_start id_end");
  }

  rpg::save(dir + "/rrt_stats.txt", data, rpg::EigenSpaceSeparatedFmt,
            "outer_iter  n_iters  best_cost  n_vert  n_edge");
}
}  // namespace act_map_exp
