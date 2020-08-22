#include "act_map_exp/quad_traj_opt.h"

namespace act_map_exp
{
const std::unordered_map<std::string, ceres::LineSearchDirectionType>
    kStrToCeresLSType = { { std::string("LBFGS"), ceres::LBFGS },
                         { std::string("BFGS"), ceres::BFGS },
                         { std::string("NONLINEAR_CONJUGATE_GRADIENT"),
                           ceres::NONLINEAR_CONJUGATE_GRADIENT },
                         { std::string("STEEPEST_DESCENT"),
                           ceres::STEEPEST_DESCENT } };

const std::unordered_map<std::string, TrajOptType> kStrToTrajOptT = {
  { std::string("Unknown"), TrajOptType::kUnknown },
  { std::string("Linear"), TrajOptType::kLinear },
  { std::string("Ceres"), TrajOptType::kCeres }
};

const std::unordered_map<std::string, TrajCeresOptType> kStrToTrajCeresOptType =
    { { std::string("Position"), TrajCeresOptType::kPosition },
      { std::string("Yaw"), TrajCeresOptType::kYaw },
      { std::string("Joint"), TrajCeresOptType::kJoint },
      { std::string("TwoStages"), TrajCeresOptType::kTwoStages } };

const std::unordered_map<std::string, TrajCeresCostType> kStrToTrajCeresCostT =
    { { std::string("PosDynamic"), TrajCeresCostType::kPosDynamic },
      { std::string("YawDynamic"), TrajCeresCostType::kYawDynamic },
      { std::string("ESDF"), TrajCeresCostType::kESDF },
      { std::string("Info"), TrajCeresCostType::kInfo } };

const std::unordered_map<TrajCeresCostType, std::string> kTrajCeresCostTToStr =
    { { TrajCeresCostType::kPosDynamic, std::string("PosDynamic") },
      { TrajCeresCostType::kYawDynamic, std::string("YawDynamic") },
      { TrajCeresCostType::kESDF, std::string("ESDF") },
      { TrajCeresCostType::kInfo, std::string("Info") } };
}  // namespace act_map_exp
