#pragma once

#include <cstdint>

#include "ava_protocol/command.h"

namespace ava {

// Parses one line (without line ending). Never fails: bad input gives CommandType::Invalid.
Command parseCommand(const char* line);

// 0 stays 0 (channel off); anything else is clamped to kMinPulseUs..kMaxPulseUs.
uint16_t clampPulse(long pulse_us);

}  // namespace ava
