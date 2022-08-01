/*
 * Copyright 2022 LeoDrive.ai, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "applanix_driver/gsof/stream_parser.h"

#include <algorithm>
#include <cstring>
#include <iterator>

namespace applanix_driver::gsof {

StreamParser::StreamParser() :
    state_(State::k_find_start) {
  constexpr std::size_t k_buf_size = 4096;
  buf_.reserve(k_buf_size);
}

std::optional<std::vector<std::byte>> StreamParser::readSome(const uint8_t *const data, std::size_t length) {
  if (buf_.size() + length > buf_.max_size()) {
    reset();
    buf_.clear();
  }

  buf_.resize(buf_.size() + length);
  std::memcpy(buf_.data() + buf_.size() - length, data, length);

  if (!isFullPacketFound()) {
    return std::nullopt;
  }

  std::size_t total_data_length = getTotalRecordLength(current_header_);
  std::vector<std::byte> result;
  result.resize(total_data_length);
  std::memcpy(&result[0], &buf_[0], total_data_length);

  // Keep parts of the buffer we still need
  buf_.erase(buf_.begin(), buf_.begin() + total_data_length);

  return result;
}

bool StreamParser::isFullPacketFound() {
  // This switch acts as a very compact state machine
  bool packet_found = false;
  auto previous_state = state_;
  switch (state_) {
    case State::k_find_start:
      state_ = !isStartTxFound() ? State::k_find_start
                                 : State::k_find_header;
      break;
    case State::k_find_header:
      state_ = !isHeaderFound() ? State::k_find_header
                                : State::k_find_end;
      break;
    case State::k_find_end:
      packet_found = isEndTxFound();
      state_ = !packet_found ? State::k_find_end : State::k_find_start;
      break;
  }

  if (previous_state != state_ && !packet_found) {
    return isFullPacketFound();
  }

  return packet_found;
}

bool StreamParser::isStartTxFound() {
  return static_cast<std::uint8_t>(buf_[0]) == START_TX;
}

bool StreamParser::isHeaderFound() {
  // Need more data
  if (buf_.size() < sizeof(record::Header)) {
    return false;
  }

  std::memcpy(&current_header_, buf_.data(), sizeof(current_header_));
  return current_header_.start_tx == START_TX;
}

bool StreamParser::isEndTxFound() {
  std::size_t footer_byte_offset = getFooterByteOffset(current_header_);

  if (buf_.size() < footer_byte_offset + sizeof(record::Footer)) {
    // Need more data
    return false;
  }

  record::Footer footer;
  std::memcpy(&footer, buf_.data() + footer_byte_offset, sizeof(record::Footer));

  // We read in the required length and it failed, reset stream parser
  return footer.end_tx == END_TX;
}

void StreamParser::reset() {
  buf_.clear();
  state_ = State::k_find_start;
}

}  // namespace applanix_driver::gsof
