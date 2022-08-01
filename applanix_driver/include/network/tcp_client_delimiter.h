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

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <boost/asio.hpp>

#include "network/ip_client.h"

namespace network {
class TcpClientDelimiter : public IpClient {
 public:
  TcpClientDelimiter() = delete;
  /**
   * A thin wrapper over a TCP socket which parses the stream using the given delimeters
   */
  TcpClientDelimiter(const std::string &ip_address, unsigned int port,
                     std::vector<std::byte> start_delimiter,
                     std::vector<std::byte> end_delimiter) noexcept;

  util::Status open() override;
  int receive() override;

 private:
  std::string start_delimiter_;
  std::string end_delimiter_;

  boost::asio::io_service io_service_;
  boost::asio::basic_streambuf<std::allocator<std::uint8_t>> streambuf_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
};
}  // namespace network
