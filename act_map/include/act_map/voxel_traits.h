#pragma once
#include <type_traits>

namespace act_map
{

namespace traits
{
template <typename T>
struct is_quad_vox : std::false_type {};
template <typename T>
struct is_vis_vox : std::false_type {};
template <typename T>
struct is_info_vox : std::false_type {};
template <typename T>
struct is_trace_vox : std::false_type {};
}

}
