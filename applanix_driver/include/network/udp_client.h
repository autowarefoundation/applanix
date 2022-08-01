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

#include <iostream>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include "network/ip_client.h"
#include "util/status.h"

namespace network {
class UdpClient : public IpClient {
 public:
  UdpClient(std::string ip_address, unsigned int port);
  util::Status open() override;
  int receive() override;

 private:
  using udp =  boost::asio::ip::udp;

  boost::asio::io_service io_service_;
  std::unique_ptr<udp::socket> socket_;
  udp::endpoint server_endpoint_;
  udp::endpoint local_endpoint_;
};
}  // namespace network
