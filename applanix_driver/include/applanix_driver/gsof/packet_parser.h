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

#include <cstdint>

#include "applanix_driver/gsof/message_parser.h"
#include "applanix_driver/parser_interface.h"

namespace applanix_driver::gsof {
class PacketParser : public ParserInterface {
 public:
  PacketParser();
  /**
   * @brief The PacketParser class expects to be parsing a report containing one or more GSOF
   * messages.
   * 
   * @param data Pointer to the start of the report (i.e. contains START_TX byte and page numbers)
   * @param length
   */
  PacketParser(const std::byte *data, std::size_t length);

  void setData(const std::byte *data, std::size_t length) override;
  [[nodiscard]] bool isValid() const override;
  [[nodiscard]] bool isSupported() const override;

  [[nodiscard]] MessageParser getMessageParser() const;

 private:
  const std::byte *messages_;

  MessageParser message_parser_;
};
}  // namespace applanix_driver::gsof
