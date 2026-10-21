#pragma once

#include <cstddef>

#include "ava_protocol/protocol_config.h"

namespace ava {

// Collects serial characters into one line. '\r' is ignored, '\n' completes the line.
class LineBuffer {
 public:
  // Adds one character. Returns true when a line is complete; read it with line() and
  // overflowed() before the next push(), which starts a new line.
  bool push(char c);

  // The completed line, without the line ending.
  const char* line() const { return buffer_; }

  // True if the completed line was longer than kMaxLineLength (its content is then incomplete).
  bool overflowed() const { return overflowed_; }

 private:
  char buffer_[kMaxLineLength + 1] = {};
  std::size_t length_ = 0;
  bool overflowed_ = false;
  bool complete_ = false;
};

}  // namespace ava
