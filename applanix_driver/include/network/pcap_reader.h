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

#include <boost/asio/buffer.hpp>
#include <pcap/pcap.h>

#include <list>
#include <optional>
#include <string>

#include "network/network.h"

namespace network {

class PcapReader {
 public:
  PcapReader(const std::string &filename, const std::string &ip_address, unsigned int port,
             network::ProtocolType protocol_type);
  ~PcapReader();
  explicit operator bool() const;
  bool readSingle(network::NonOwningBuffer *payload);

  /**
   * Reads the whole pcap file into memory, stripping protocol data and keeping only the payload.
   * XXX Make sure you have enough RAM
   */
  std::list<std::vector<std::byte>> reallAll();

 private:
  network::ProtocolType protocol_type_;
  char err_buf_[512];
  int result_;
  pcap_t *handle_;
  bpf_program compiled_filter_;

  NonOwningBuffer getPcapPayload(const uint8_t *data, std::size_t length, ProtocolType pt);
  NonOwningBuffer getTcpPayload(const uint8_t *data, std::size_t length);
  NonOwningBuffer getUdpPayload(const uint8_t *data, std::size_t length);
};

}  // namespace network
