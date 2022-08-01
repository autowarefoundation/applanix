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

#include "network/tcp_client.h"

namespace network {

TcpClient::TcpClient(const std::string &ip_address, unsigned int port)
    : IpClient(ip_address, port),
      socket_(nullptr) {

}

util::Status TcpClient::open() {
  using namespace boost::asio::ip;
  socket_ = std::make_unique<tcp::socket>(io_service_);

  boost::system::error_code ec;
  socket_->connect(tcp::endpoint(boost::asio::ip::address::from_string(ip_address_), port_),
                   ec);

  boost::asio::ip::tcp::no_delay no_delay(true);
  socket_->set_option(no_delay);

  if (!ec) {
    return util::Status(util::ErrorCode::OK);
  }

  return util::Status(util::ErrorCode::CONNECTION_ERROR, ec.message());
}

int TcpClient::receive() {
  boost::system::error_code error;
  size_t bytes_read = socket_->read_some(boost::asio::buffer(payload_buffer_), error);

  if (error) {
    return -1;
  }

  return bytes_read;
}

}  // namespace network
