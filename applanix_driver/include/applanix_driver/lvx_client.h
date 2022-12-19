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

#include <atomic>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <stdexcept>
#include "applanix_driver/gsof/stream_parser.h"
#include "applanix_driver/gsof/message_parser.h"
#include "network/ip_client.h"
#include "util/status.h"

namespace applanix_driver {

/**
 * This client is meant to connect to products communicating using Trimble's Trimcomm/GSOF packets
 * over TCP. Should also be compatible with APX/LVX products.
 */
class LvxClient {
 public:
  using MessageCallback = std::function<void(const gsof::Message &)>;
  struct unsupported_callback_error : public std::runtime_error {
    unsupported_callback_error() : std::runtime_error("Unable to dispacth callback, need to add "
                                                      "dispatch in LvxClient::callbackDispatch") {}
  };

  LvxClient() = delete;
  LvxClient(std::string ip_address, unsigned int port);
  ~LvxClient();
  LvxClient(const LvxClient &) = delete;
  LvxClient &operator=(const LvxClient &) = delete;

  util::Status start();
  void stop();

  std::vector<MessageCallback>::iterator registerCallback(gsof::Id id,
                                                          const MessageCallback &callback);

 private:
  std::unique_ptr<std::thread> tcp_thread_;
  std::atomic_bool keep_running_;

  std::unique_ptr<network::IpClient> tcp_client_;

  std::unordered_map<gsof::Id, std::vector<MessageCallback>> message_callbacks_;
  gsof::StreamParser gsof_stream_parser_;

  void runTcpConnection();
  void grabAndParseTcp();

};
}  // namespace applanix_driver
