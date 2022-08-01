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

#include "network/pcap_reader.h"

#include <sstream>
#include <iostream>

namespace network {

PcapReader::PcapReader(const std::string &filename,
                       const std::string &ip_address,
                       unsigned int port,
                       network::ProtocolType protocol_type) :
    protocol_type_(protocol_type),
    err_buf_{0},
    result_(1),
    handle_(nullptr) {
  handle_ = pcap_open_offline(filename.c_str(), err_buf_);

  if (handle_ == nullptr) {
    return;
  }

  // Filter all packets not comping from
  std::stringstream filter_expression;
  filter_expression << "src net " << ip_address << " and ";
  switch (protocol_type_) {
    case network::ProtocolType::TCP:
      filter_expression << "tcp port " << port;
      break;
    case network::ProtocolType::UDP:
      filter_expression << "udp port " << port;
      break;
    default:
      throw std::invalid_argument("Unhandled ip protocol type.");
  }

  result_ = pcap_compile(handle_,
                         &compiled_filter_,
                         filter_expression.str().c_str(),
                         1,  // 1 means do optimization
                         PCAP_NETMASK_UNKNOWN);

  if (result_ < 0) {
    return;
  }

  result_ = pcap_setfilter(handle_, &compiled_filter_);
  if (result_ < 0) {
    return;
  }
}

PcapReader::~PcapReader() {
  if (handle_ != nullptr) {
    pcap_close(handle_);
    handle_ = nullptr;
  }
}

PcapReader::operator bool() const {
  return handle_ != nullptr && result_ >= 0;
}

bool PcapReader::readSingle(network::NonOwningBuffer *payload) {
  struct pcap_pkthdr *header;
  const u_char *pkt_data;
  result_ = pcap_next_ex(handle_, &header, &pkt_data);

  if (result_ < 1) {
    return false;
  }

  *payload = getPcapPayload(pkt_data, header->caplen, protocol_type_);

  return true;
}

NonOwningBuffer PcapReader::getTcpPayload(const uint8_t *data, const std::size_t length) {
  IpHeader ip_header;
  std::memcpy(&ip_header, data + EthernetHeader::SIZE, sizeof ip_header);

  // Invalid header
  if (ip_header.getHeaderLength() < IpHeader::MIN_LEN)
    return {nullptr, 0};

  TcpHeader tcp_header;
  std::memcpy(&tcp_header,
              data + EthernetHeader::SIZE + ip_header.getHeaderLength(),
              sizeof tcp_header);

  // Invalid header
  auto data_offset = tcp_header.getDataOffset();
  if (data_offset < TcpHeader::MIN_SIZE || data_offset > TcpHeader::MAX_SIZE)
    return {nullptr, 0};

  const size_t
      header_size = EthernetHeader::SIZE + ip_header.getHeaderLength() + tcp_header.getDataOffset();
  const uint8_t *payload = data + header_size;

  return {payload, length - header_size};
}

NonOwningBuffer PcapReader::getUdpPayload(const uint8_t *data, const std::size_t length) {
  IpHeader ip_header;
  std::memcpy(&ip_header, data + EthernetHeader::SIZE, sizeof ip_header);

  // Invalid header
  if (ip_header.getHeaderLength() < 20)
    return {nullptr, 0};

  UdpHeader udp_header;
  std::memcpy(&udp_header,
              data + EthernetHeader::SIZE + ip_header.getHeaderLength(),
              sizeof(udp_header));

  if (udp_header.length < UdpHeader::MIN_SIZE) return {nullptr, 0};

  // TODO(Andre) Make use of checksum

  const size_t
      header_size = EthernetHeader::SIZE + ip_header.getHeaderLength() + sizeof(udp_header);
  const uint8_t *payload = data + header_size;

  return {payload, length - header_size};
}

NonOwningBuffer PcapReader::getPcapPayload(const uint8_t *data, const std::size_t length,
                                           const ProtocolType pt) {
  if (pt == ProtocolType::TCP) {
    return getTcpPayload(data, length);
  } else if (pt == ProtocolType::UDP) {
    return getUdpPayload(data, length);
  }

  return {nullptr, 0};
}

std::list<std::vector<std::byte>> PcapReader::reallAll() {
  std::list<std::vector<std::byte>> data_payloads;

  for(NonOwningBuffer buf{nullptr, 0}; readSingle(&buf);) {
    std::vector<std::byte> payload(buf.length);
    std::memcpy(payload.data(), buf.data, buf.length);
    data_payloads.emplace_back(std::move(payload));
  }

  return data_payloads;
}

}  // namespace network
