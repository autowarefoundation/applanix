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
#include <string>

namespace applanix_driver::gsof {
// @formatter off
static constexpr uint8_t START_TX = 0x02;
static constexpr uint8_t END_TX = 0x03;

static constexpr uint8_t GENOUT = 0x40;

static constexpr uint8_t GENOUT_BYTE_STX = 0;
static constexpr uint8_t GENOUT_BYTE_STATUS = 1;
static constexpr uint8_t GENOUT_BYTE_PACKET_TYPE = 2;
static constexpr uint8_t GENOUT_BYTE_LENGTH = 3;
static constexpr uint8_t GENOUT_BYTE_TRANS_NUM = 4;
static constexpr uint8_t GENOUT_BYTE_PAGE_IDX = 5;
static constexpr uint8_t GENOUT_BYTE_MAX_PAGE_IDX = 6;

// Because the data length count starts immediate after, the count includes 3 bytes of the header.
static constexpr uint8_t NUM_HEADER_BYTES_IN_DATA_LENGTH = 3;
// @formatter on

namespace record {
#pragma pack(push, 1)
struct Header {
  uint8_t start_tx;
  uint8_t status;
  uint8_t type;
  uint8_t data_len;       // Length of the payload which starts immediately after the data_len byte
                          // This means it includes the last 3 bytes of the header but not the footer
  uint8_t tx_num;
  uint8_t page_idx;
  uint8_t max_page_idx;
};

struct Footer {
  uint8_t checksum;
  uint8_t end_tx;
};
#pragma pack(pop)

/**
 * Gets the total number of bytes of a GSOF record including the Header and Footer
 */
inline std::size_t getTotalRecordLength(const Header& header) {
  constexpr size_t k_header_bytes_not_in_data_len = sizeof(Header) -  NUM_HEADER_BYTES_IN_DATA_LENGTH;
  return header.data_len + k_header_bytes_not_in_data_len + sizeof(Footer);
}

/**
 * Get the position of the first footer byte (0-indexed) starting from the START_TX byte
 */
inline std::size_t getFooterByteOffset(const Header& header) {
  return sizeof(Header) - NUM_HEADER_BYTES_IN_DATA_LENGTH + header.data_len;
}

}  // namespace record
}  // namespace applanix_driver::gsof
