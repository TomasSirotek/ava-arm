#pragma once

#include <cstdint>

#include "ava_protocol/protocol_config.h"

namespace ava {

// Protocol v1, one ASCII line per command:
//   "?"                   -> Query
//   "P u0 u1 u2 u3 u4 u5" -> SetPulses, microseconds per channel, 0 = off
//   "D"                   -> Disable (all channels off)
enum class CommandType { Empty, Query, SetPulses, Disable, Invalid };

struct Command {
  CommandType type = CommandType::Empty;
  uint16_t pulses_us[kChannelCount] = {};  // SetPulses only; already clamped
  const char* error = "";                  // Invalid only: short reason for the "ERR" reply
};

}  // namespace ava
