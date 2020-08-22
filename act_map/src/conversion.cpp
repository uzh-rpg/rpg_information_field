#include "act_map/conversion.h"

namespace act_map
{
template <>
void serializeElem<>(const double& from, uint32_t* to)
{
  float v = static_cast<float>(from);
  (*to) = *(reinterpret_cast<uint32_t*>(&v));
}

template <>
void serializeElem<>(const double& from, uint64_t* to)
{
  (*to) = *(reinterpret_cast<const uint64_t*>(&from));
}

template <>
void deserializeElem<>(const uint32_t& from, double* to)
{
  float v;
  memcpy(&v, &from, sizeof(uint32_t));
  (*to) = static_cast<double>(v);
}

template <>
void deserializeElem<>(const uint64_t& from, double* to)
{
  memcpy(to, &from, sizeof(uint64_t));
}

}
