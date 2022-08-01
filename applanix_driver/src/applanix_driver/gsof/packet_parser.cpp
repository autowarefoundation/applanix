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

#include "applanix_driver/gsof/packet_parser.h"

#include <cstring>

#include "applanix_driver/gsof/gsof.h"

namespace applanix_driver::gsof {
PacketParser::PacketParser() : PacketParser(nullptr, 0) {
}

PacketParser::PacketParser(const std::byte *data, const size_t length) {
  PacketParser::setData(data, length);
}

void PacketParser::setData(const std::byte *data, const size_t length) {
  data_ = data;
  messages_ = data_ + sizeof(gsof::record::Header);
  length_ = length;
}

bool PacketParser::isValid() const {
  gsof::record::Header header;
  std::memcpy(&header, data_, sizeof(header));

  if (header.start_tx != gsof::START_TX)
    return false;

  // Don't support packets other than the general report packet.
  if (header.type != gsof::GENOUT)
    return false;

  gsof::record::Footer footer;
  std::memcpy(&footer, data_ + length_ - sizeof(footer), sizeof(footer));

  if (footer.end_tx != gsof::END_TX)
    return false;

  unsigned int checksum = header.status + header.type + header.data_len;
  constexpr size_t CHECKSUM_DATA_START = 4;  // (1) stx + (1) status + (1) type + (1) data_len
  for (size_t i = CHECKSUM_DATA_START; i < header.data_len + CHECKSUM_DATA_START; ++i) {
    checksum += static_cast<unsigned int>(*(data_ + i));
  }

  return checksum % 256 == footer.checksum;
}

bool PacketParser::isSupported() const {
  return getMessageParser().isSupported();
}

MessageParser PacketParser::getMessageParser() const {
  const size_t message_payload_length = length_ - sizeof(gsof::record::Header) - sizeof(gsof::record::Footer);
  return MessageParser(messages_, message_payload_length);
}

}  // namespace applanix_driver::gsof
