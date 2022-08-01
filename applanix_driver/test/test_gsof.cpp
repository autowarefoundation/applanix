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

#include <applanix_driver/gsof/packet_parser.h>
#include <applanix_driver/gsof/stream_parser.h>
#include <gtest/gtest.h>

#include <pcap/pcap.h>

#include <chrono>
#include <random>

#include "network/pcap_reader.h"

using applanix_driver::gsof::PacketParser;
using applanix_driver::gsof::Message;

static constexpr char k_apx18_ip_addr[] = "172.16.40.104";
static constexpr int k_apx18_port = 5018;

TEST(GsofParsingTest, fullNavigationInfoGsof49) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_fullnavinfo_single.pcapng", k_apx18_ip_addr,
                                  k_apx18_port, network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  PacketParser gsof_parser(reinterpret_cast<const std::byte *>(payload.data), payload.length);
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, 0x31);
  ASSERT_EQ(it->getHeader().length, 0x68);  // Read our ICD not the GSOF stuff on the web

  using namespace applanix_driver::gsof;
  auto solution = it->as<InsSolution>();
  ASSERT_EQ(solution.gps_time.week, 2080); // week of nov 17th 2019
  // Guesstimated google maps lat long of Applanix office :)
  ASSERT_NEAR(solution.lla.latitude, 43.86, 1e-2);
  ASSERT_NEAR(solution.lla.longitude, -79.38, 1e-2);

  ASSERT_NE(solution.status.getImuAlignmentStatus(), Status::ImuAlignmentStatus::UNKNOWN);
  ASSERT_NE(solution.status.getGnssStatus(), Status::GnssStatus::UNKNOWN);
}

TEST(GsofParsingTest, fullNavigationRmsGsof50) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_fullrmsinfo_single.pcapng", k_apx18_ip_addr,
                                  k_apx18_port, network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  PacketParser gsof_parser(reinterpret_cast<const std::byte *>(payload.data), payload.length);
  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());

  auto it = message_parser.begin();
  using namespace applanix_driver::gsof;
  auto rms = it->as<InsSolutionRms>();
  ASSERT_EQ(rms.gps_time.week, 2080);

  ASSERT_NE(rms.status.getImuAlignmentStatus(), Status::ImuAlignmentStatus::UNKNOWN);
  ASSERT_NE(rms.status.getGnssStatus(), Status::GnssStatus::UNKNOWN);

  ++it;
  ASSERT_EQ(it, message_parser.end());
}

TEST(GsofParsingTest, iterator) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_fullnavinfo_single.pcapng", k_apx18_ip_addr,
                                  k_apx18_port, network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  PacketParser gsof_parser(reinterpret_cast<const std::byte *>(payload.data), payload.length);

  using namespace applanix_driver;
  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, gsof::GSOF_ID_49_INS_FULL_NAV);
  ++it;
  ASSERT_EQ(it->getHeader().type, gsof::GSOF_ID_50_INS_RMS);
  ++it;
  ASSERT_EQ(it, message_parser.end());
}

TEST(GsofParsingTest, streamParser) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_fullnavinfo_single.pcapng", k_apx18_ip_addr,
                                  k_apx18_port, network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distribution(1, 512);

  std::array<std::uint8_t, 512> buf{};
  applanix_driver::gsof::StreamParser stream_parser;
  std::optional<std::vector<std::byte>> result = std::nullopt;
  // Pass on random sized buffers to the stream parser
  for (std::size_t bytes_read = 0; bytes_read < payload.length;) {
    int bytes_to_read = std::min(distribution(gen), static_cast<int>(payload.length - bytes_read));
    std::memcpy(buf.data(), payload.data + bytes_read, bytes_to_read);
    bytes_read += bytes_to_read;

    result = stream_parser.readSome(buf.data(), bytes_to_read);

    if (bytes_read != payload.length) {
      ASSERT_FALSE(result.has_value());
    } else {
      // Once all the bytes are read
      ASSERT_TRUE(result.has_value());
    }
  }

  // Now prove that you can parse the resulting vector of bytes using the packet parser
  ASSERT_TRUE(result.has_value());  // For -Wmaybe-uninitialized

  PacketParser gsof_parser(result->data(), result->size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  using namespace applanix_driver;
  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, gsof::GSOF_ID_49_INS_FULL_NAV);
  ++it;
  ASSERT_EQ(it->getHeader().type, gsof::GSOF_ID_50_INS_RMS);
  ++it;
  ASSERT_EQ(it, message_parser.end());
}

class StreamingGsofTest : public ::testing::Test {
 public:
  StreamingGsofTest() : pcap_reader_("apx_18_fullnav_fullrmsinfo.pcapng", k_apx18_ip_addr,
                                     k_apx18_port, network::ProtocolType::TCP),
                        tcp_packet_({nullptr, 0}),
                        bytes_read_of_curr_packet_(0),
                        rd_(),
                        generator_(rd_()),
                        distribution_(1, 25),
                        buf_({}) {
    pcap_reader_.readSingle(&tcp_packet_);
  }

  std::optional<network::NonOwningBuffer> getData() {
    if (tcp_packet_.data == nullptr) {
      return std::nullopt;
    }

    int bytes_left_in_curr_pkt = tcp_packet_.length - bytes_read_of_curr_packet_;
    int bytes_to_read = getRandom();
    const std::uint8_t *curr_data_ptr = tcp_packet_.data + bytes_read_of_curr_packet_;

    if (bytes_to_read < bytes_left_in_curr_pkt) {
      std::memcpy(buf_.data(), curr_data_ptr, bytes_to_read);
      bytes_read_of_curr_packet_ += bytes_to_read;
    } else if (bytes_to_read == bytes_left_in_curr_pkt) {
      std::memcpy(buf_.data(), curr_data_ptr, bytes_to_read);
      readNextPacket();
    } else if (bytes_to_read > bytes_left_in_curr_pkt) {
      std::memcpy(buf_.data(), curr_data_ptr, bytes_left_in_curr_pkt);
      if(readNextPacket()) {
        // Need to reset ptr because we read in a new packet
        curr_data_ptr = tcp_packet_.data + bytes_read_of_curr_packet_;
        int remainder = bytes_to_read - bytes_left_in_curr_pkt;
        std::memcpy(buf_.data() + bytes_left_in_curr_pkt, curr_data_ptr, remainder);
        bytes_read_of_curr_packet_ += remainder;
      }
    }

    return network::NonOwningBuffer({reinterpret_cast<const std::uint8_t *>(buf_.data()),
                                     static_cast<std::size_t>(bytes_to_read)});
  }

 protected:
  network::PcapReader pcap_reader_;
  network::NonOwningBuffer tcp_packet_;
  std::size_t bytes_read_of_curr_packet_;

 private:
  std::random_device rd_;
  std::mt19937 generator_;
  std::uniform_int_distribution<> distribution_;
  std::array<std::byte, 32> buf_;

  int getRandom() {
    return distribution_(generator_);
  }

  bool readNextPacket() {
    bytes_read_of_curr_packet_ = 0;
    bool result = pcap_reader_.readSingle(&tcp_packet_);
    if (!result) {
      tcp_packet_.data = nullptr;
      tcp_packet_.length = 0;
    }
    return result;
  }
};

TEST_F(StreamingGsofTest, streamParserWithOverlap) {
  applanix_driver::gsof::StreamParser stream_parser;
  std::optional<network::NonOwningBuffer> maybe_data = std::nullopt;
  for(maybe_data = getData(); maybe_data.has_value(); maybe_data = getData()) {
    auto maybe_gsof_record = stream_parser.readSome(maybe_data->data, maybe_data->length);
    if (maybe_gsof_record) {
      PacketParser packet_parser(maybe_gsof_record->data(), maybe_gsof_record->size());
      ASSERT_TRUE(packet_parser.isValid());
      ASSERT_TRUE(packet_parser.isSupported());

      // Every record in this pcap contains a full ins nav and the associated RMS error values
      auto msg_parser = packet_parser.getMessageParser();
      ASSERT_TRUE(msg_parser.isSupported());
      ASSERT_TRUE(msg_parser.isValid());

      auto it = msg_parser.begin();
      ASSERT_EQ(it->getHeader().type, applanix_driver::gsof::GSOF_ID_49_INS_FULL_NAV);
      ++it;
      ASSERT_EQ(it->getHeader().type, applanix_driver::gsof::GSOF_ID_50_INS_RMS);
      ++it;
      ASSERT_EQ(it, msg_parser.end());
    }
  }
}
