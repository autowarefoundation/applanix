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

#include "applanix_driver/gsof/gsof.h"
#include "applanix_driver/gsof/packet_parser.h"
#include "applanix_driver/lvx_client.h"

#include "network/tcp_client.h"

namespace applanix_driver {

LvxClient::LvxClient(std::string ip_address,
                     unsigned int port) :
    keep_running_(true) {
  tcp_client_ = std::make_unique<network::TcpClient>(ip_address, port);
}

LvxClient::~LvxClient() {
  stop();
  if (tcp_thread_->joinable()) {
    tcp_thread_->join();
  }
}

util::Status LvxClient::start() {
  util::Status status = tcp_client_->open();
  if (!status) return status;

  tcp_thread_ = std::make_unique<std::thread>(&LvxClient::runTcpConnection, this);
  return status;
}

void LvxClient::stop() {
  keep_running_ = false;
}

std::vector<LvxClient::MessageCallback>::iterator
LvxClient::registerCallback(gsof::Id id, const LvxClient::MessageCallback &callback) {
  if (message_callbacks_.count(id) == 0) {
    message_callbacks_[id] = std::vector<MessageCallback>();
  }

  message_callbacks_[id].push_back(callback);

  return --message_callbacks_[id].end();
}

void LvxClient::runTcpConnection() {
  while (keep_running_.load()) {
    grabAndParseTcp();
  }
}

void LvxClient::grabAndParseTcp() {
  int bytes_rcvd = tcp_client_->receive();
  if (bytes_rcvd < 1) return;

  auto &&buffer = tcp_client_->getBuffer();
  std::optional<std::vector<std::byte>> maybe_gsof_record = gsof_stream_parser_.readSome(buffer.data(), bytes_rcvd);

  if (!maybe_gsof_record) {
    return;
  }

  gsof::PacketParser packet_parser(maybe_gsof_record->data(), maybe_gsof_record->size());

  if (!packet_parser.isValid()) {
    return;
  }

  for (const gsof::Message &message : packet_parser.getMessageParser()) {
    auto message_callbacks = message_callbacks_.find(message.getHeader().type);
    if (message_callbacks == message_callbacks_.end()) {
      continue;
    }

    for (const MessageCallback &callback : message_callbacks->second) {
      callback(message);
    }
  }
}

}  // namespace applanix_driver
