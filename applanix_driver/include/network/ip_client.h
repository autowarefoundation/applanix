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

#include <cstdint>
#include <string>
#include <utility>
#include "util/status.h"

namespace network {

class IpClient {
 public:
  static constexpr size_t MAX_BUF_SIZE = 4096;
  using Buffer = std::array<uint8_t, MAX_BUF_SIZE>;

  IpClient(std::string ip_address, unsigned int port) : ip_address_(std::move(ip_address)), port_(port) {}
  virtual util::Status open() = 0;
  virtual int receive() = 0;
  const Buffer &getBuffer() const { return payload_buffer_; }

 protected:
  std::string ip_address_;
  unsigned int port_;
  Buffer payload_buffer_;

 private:
};

}  // namespace network
