#pragma once

#include <cstddef>
#include <cstdint>

// Values the ROS driver (ros2_ws/src/ava_hardware) relies on. Change them together with it.
namespace ava {

constexpr int kProtocolVersion = 1;

// PCA9685 channels 0..5 = joints J1..J6 (channel map: README.md and ava.control.xacro).
constexpr uint8_t kChannelCount = 6;

// Safety net: non-zero pulses are clamped to this range (the ROS driver clamps first).
constexpr uint16_t kMinPulseUs = 500;
constexpr uint16_t kMaxPulseUs = 2500;

// Longest accepted command line, without the line ending ("P 2500 2500 2500 2500 2500 2500" = 31).
constexpr std::size_t kMaxLineLength = 64;

}  // namespace ava
