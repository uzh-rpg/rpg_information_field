#include "act_map/quadratic_factor_voxel.h"

namespace act_map
{
template <>
constexpr size_t QuadInfoVoxel::kSerializationSize = 468u;
template <>
constexpr size_t QuadTraceVoxel::kSerializationSize = 13u;
}

namespace act_map
{
namespace voxblox
{
namespace voxel_types
{
const std::string kQuadTrace = "quad_trace";
const std::string kQuadInfo = "quad_info";
}  // voxel types
}  // namespace voxblox
}  // namespace act_map
