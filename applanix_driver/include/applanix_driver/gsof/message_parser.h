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
#include <set>
#include "applanix_driver/parser_interface.h"
#include "applanix_driver/gsof/message.h"

namespace applanix_driver::gsof {

class MessageParser : public ParserInterface {
 public:
  MessageParser();
  MessageParser(const std::byte *data, const std::size_t length);

  void setData(const std::byte *data, std::size_t length) override;
  bool isValid() const override;
  bool isSupported() const override;

  class Iterator {
   public:
    friend class MessageParser;
    // Declarations for std iterator compatibility
    using iterator_category = std::forward_iterator_tag;
    using value_type = gsof::Message;
    using difference_type = int;
    using pointer = value_type *;
    using reference = value_type &;

    Iterator(const std::byte *const data_begin, std::size_t length);

    reference operator*();
    pointer operator->();
    Iterator &operator++();
    bool operator==(const Iterator &rhs) const;
    bool operator!=(const Iterator &rhs) const;

   private:
    const std::byte *const data_begin_;
    const std::byte *data_;
    const std::size_t length_;
    std::size_t current_offset_;
    Message current_message_;

    Iterator &invalidate();
  };

  Iterator begin();
  Iterator end();

 private:
  static const std::set<std::uint8_t> supported_messages_;
  const std::byte *data_;
  std::size_t length_;

  [[nodiscard]] inline bool isMessageSupported(std::uint8_t id) const;
};

}  // namespace applanix_driver::gsof
