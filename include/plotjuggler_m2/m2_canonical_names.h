#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <string_view>

namespace plotjuggler_m2
{

inline constexpr std::size_t kM2JointCount = 12;
inline constexpr std::size_t kM2LegCount = 4;
inline constexpr std::size_t kM2JointsPerLeg = 3;
inline constexpr std::size_t kM2AxesCount = 6;
inline constexpr std::size_t kM2ButtonCount = 12;

// Standard Svan M2 URDF joint names indexed 0..11
// Ordering convention: FR -> FL -> RR -> RL, each leg [HAA, HFE, KFE]
inline constexpr std::array<std::string_view, kM2JointCount> kJointNames = {
    "FR_hip_joint",   // 0: Front-Right Abduction (HAA)
    "FR_thigh_joint", // 1: Front-Right Hip Flexion (HFE)
    "FR_calf_joint",  // 2: Front-Right Knee (KFE)
    "FL_hip_joint",   // 3: Front-Left Abduction (HAA)
    "FL_thigh_joint", // 4: Front-Left Hip Flexion (HFE)
    "FL_calf_joint",  // 5: Front-Left Knee (KFE)
    "RR_hip_joint",   // 6: Rear-Right Abduction (HAA)
    "RR_thigh_joint", // 7: Rear-Right Hip Flexion (HFE)
    "RR_calf_joint",  // 8: Rear-Right Knee (KFE)
    "RL_hip_joint",   // 9: Rear-Left Abduction (HAA)
    "RL_thigh_joint", // 10: Rear-Left Hip Flexion (HFE)
    "RL_calf_joint"   // 11: Rear-Left Knee (KFE)
};

inline constexpr std::array<std::string_view, kM2LegCount> kLegNames = {
    "FR", "FL", "RR", "RL"
};

inline constexpr std::array<std::string_view, kM2JointsPerLeg> kJointTypes = {
    "hip", "thigh", "calf"
};

// JoyData axis semantic names (with known Svan M2 hardware inversions noted)
inline constexpr std::array<std::string_view, kM2AxesCount> kJoyAxisNames = {
    "lateral",      // Axis 0: Left / Right (Inverted)
    "longitudinal", // Axis 1: Forward / Backward (Inverted: -1.0 = max forward)
    "axis_2",       // Axis 2: Unused / Auxiliary
    "yaw",          // Axis 3: Rotation about Z (Inverted)
    "body_height",  // Axis 4: Body height (Inverted)
    "step_height"   // Axis 5: Step clearance height (Normal 0.0 - 1.0)
};

// JoyData button semantic names per state machine transition rules
inline constexpr std::array<std::string_view, kM2ButtonCount> kJoyButtonNames = {
    "fixed_stand",  // Button 0: State Fixed Stand
    "sleep",        // Button 1: State Sleep
    "move",         // Button 2: State Move (Locomotion)
    "freestand",    // Button 3: State Free Stand
    "button_4",
    "button_5",
    "button_6",
    "button_7",
    "button_8",
    "button_9",
    "button_10",
    "button_11"
};

// Default CycloneDDS and ROS 2 topic names
inline constexpr std::string_view kDdsSensorTopic = "rt/m2_metal/hw/sensor_data";
inline constexpr std::string_view kRosSensorTopic = "/m2_metal/hw/sensor_data";

inline constexpr std::string_view kDdsJointCommandTopic = "rt/m2_metal/hw/joint_command";
inline constexpr std::string_view kRosJointCommandTopic = "/m2_metal/hw/joint_command";

inline constexpr std::string_view kDdsJoystickTopic = "rt/mission/joystick_data";
inline constexpr std::string_view kRosJoystickTopic = "/mission/joystick_data";

inline std::string formatIndex(std::size_t index)
{
  return (index < 10 ? "0" : "") + std::to_string(index);
}


inline std::string legJointAlias(std::size_t index, std::string_view field)
{
  if (index >= kM2JointCount) return "";
  return std::string("legs/") + std::string(kLegNames[index / kM2JointsPerLeg]) + "/" +
         std::string(kJointTypes[index % kM2JointsPerLeg]) + "/" + std::string(field);
}

} // namespace plotjuggler_m2
