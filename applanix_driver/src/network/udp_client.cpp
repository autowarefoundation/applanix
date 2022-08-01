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

#include "network/udp_client.h"

namespace network {

UdpClient::UdpClient(std::string ip_address, unsigned int port) :
    IpClient(ip_address, port) {
  payload_buffer_.fill(0);
  server_endpoint_ = udp::endpoint(boost::asio::ip::address_v4::from_string(ip_address),
                                   port_);
  local_endpoint_ = udp::endpoint(boost::asio::ip::address_v4::any(),
      port_);
}

util::Status UdpClient::open() {
  socket_ = std::make_unique<udp::socket>(io_service_);
  boost::system::error_code ec;
  socket_->open(local_endpoint_.protocol(), ec);
  socket_->bind(local_endpoint_);

  if (!ec) {
    return util::Status();
  } else {
    return util::Status(util::ErrorCode::CONNECTION_ERROR, ec.message());
  }
}

int UdpClient::receive() {
  return socket_->receive_from(boost::asio::buffer(payload_buffer_), server_endpoint_);
}

}  // namespace network
