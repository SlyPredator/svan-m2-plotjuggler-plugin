#include "plotjuggler_m2/m2_canonical_names.h"

#include <algorithm>

namespace plotjuggler_m2
{

std::string formatIndex(std::size_t index)
{
  if (index < 10)
  {
    return "0" + std::to_string(index);
  }
  return std::to_string(index);
}

int jointNameToIndex(std::string_view name)
{
  for (std::size_t i = 0; i < kJointNames.size(); ++i)
  {
    if (kJointNames[i] == name)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::string legJointAlias(std::size_t index, std::string_view field)
{
  if (index >= kM2JointCount)
  {
    return "";
  }
  const std::size_t leg_idx = index / kM2JointsPerLeg;
  const std::size_t joint_idx = index % kM2JointsPerLeg;
  return std::string("legs/") + std::string(kLegNames[leg_idx]) + "/" +
         std::string(kJointTypes[joint_idx]) + "/" + std::string(field);
}

} // namespace plotjuggler_m2
