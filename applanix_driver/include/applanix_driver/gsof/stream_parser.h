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

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "applanix_driver/gsof/gsof.h"

namespace applanix_driver::gsof {

/**
 * Given a sequence of bytes, accumulates bytes into a buffer until the buffer contains a GSOF
 * packet. Intended for use with streaming input.
 */
class StreamParser {
 public:
  StreamParser();

  std::optional<std::vector<std::byte>> readSome(const std::uint8_t *const data, std::size_t length);

 private:
  enum class State { k_find_start, k_find_header, k_find_end };

  std::vector<std::byte> buf_;
  State state_;
  record::Header current_header_;

  bool isFullPacketFound();

  // Reset the state machine
  void reset();

  bool isStartTxFound();
  bool isHeaderFound();
  bool isEndTxFound();

};

}  // namespace applanix_driver::gsof
