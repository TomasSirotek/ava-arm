#include "ava_protocol/line_buffer.h"

namespace ava {

bool LineBuffer::push(char c) {
  if (complete_) {  // the previous line was read: start a new one
    length_ = 0;
    overflowed_ = false;
    complete_ = false;
  }
  if (c == '\r') {
    return false;
  }
  if (c == '\n') {
    buffer_[length_] = '\0';
    complete_ = true;
    return true;
  }
  if (length_ < kMaxLineLength) {
    buffer_[length_++] = c;
  } else {
    overflowed_ = true;
  }
  return false;
}

}  // namespace ava
