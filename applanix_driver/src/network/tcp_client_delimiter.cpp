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

#include "network/tcp_client_delimiter.h"

#include <algorithm>

namespace network {
TcpClientDelimiter::TcpClientDelimiter(const std::string &ip_address, unsigned int port,
                                       std::vector<std::byte> start_delimiter,
                                       std::vector<std::byte> end_delimiter) noexcept
    : IpClient(ip_address, port),
      streambuf_(4096) {
  start_delimiter_.reserve(start_delimiter.size());
  end_delimiter_.reserve(end_delimiter.size());

  std::transform(start_delimiter.begin(), start_delimiter.end(), std::back_inserter(start_delimiter_),
                 [](std::byte b) -> char {
                   return static_cast<char>(b);
                 });
  std::transform(end_delimiter.begin(), end_delimiter.end(), std::back_inserter(end_delimiter_),
                 [](std::byte b) {
                   return static_cast<char>(b);
                 });

  payload_buffer_.fill(0U);
}

util::Status TcpClientDelimiter::open() {
  using namespace boost::asio::ip;
  socket_ = std::make_unique<tcp::socket>(io_service_);

  boost::system::error_code ec;
  socket_->connect(tcp::endpoint(boost::asio::ip::address::from_string(ip_address_), port_),
                   ec);
  if (ec) {
    return util::Status(util::ErrorCode::CONNECTION_ERROR, ec.message());
  }

  boost::asio::ip::tcp::no_delay no_delay(true);
  socket_->set_option(no_delay);
  return util::Status();
}

int TcpClientDelimiter::receive() {
  boost::system::error_code error;

  // Read until we find the start delimiter and drop everything
  size_t bytes_read = boost::asio::read_until(*socket_, streambuf_, start_delimiter_, error);

  if (error) {
    return -1;
  }

  auto it = std::copy(start_delimiter_.begin(), start_delimiter_.end(), payload_buffer_.begin());
  streambuf_.consume(bytes_read);

  // Read until we find the end delimiter and copy to our payload buffer
  bytes_read = boost::asio::read_until(*socket_, streambuf_, end_delimiter_, error);
  std::copy(boost::asio::buffers_begin(streambuf_.data()),
            boost::asio::buffers_begin(streambuf_.data()) + bytes_read,
            it);
  streambuf_.consume(bytes_read);

  if (error) { return -1; }

  return bytes_read + start_delimiter_.size();
}

}  // namespace network
