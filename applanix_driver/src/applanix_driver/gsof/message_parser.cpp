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

#include "applanix_driver/gsof/message_parser.h"

#include <cstring>

#include "applanix_driver/gsof/message.h"

namespace applanix_driver::gsof {

const std::set<std::uint8_t> MessageParser::supported_messages_ = {
    GSOF_ID_49_INS_FULL_NAV,
    GSOF_ID_50_INS_RMS,
};

MessageParser::MessageParser() : MessageParser(nullptr, 0) {}

MessageParser::MessageParser(const std::byte *data, std::size_t length) : data_(data), length_(length) { }

void MessageParser::setData(const std::byte *data, std::size_t length) {
  data_ = data;
  length_ = length;
}

bool MessageParser::isValid() const {
  Header header;
  std::memcpy(&header, data_, sizeof(header));
  return isMessageSupported(header.type);
}

bool MessageParser::isSupported() const {
  // GSOF messages don't seem to have an end?
  return isValid();
}

bool MessageParser::isMessageSupported(std::uint8_t id) const {
  return supported_messages_.count(id) > 0;
}

MessageParser::Iterator MessageParser::begin() {
  return Iterator(data_, length_);
}

MessageParser::Iterator MessageParser::end() {
  return this->begin().invalidate();
}

MessageParser::Iterator::Iterator(const std::byte *const data_begin, const std::size_t length) :
    data_begin_(data_begin),
    data_(data_begin_),
    length_(length),
    current_offset_(0),
    current_message_(data_begin, length) {}

MessageParser::Iterator::reference MessageParser::Iterator::operator*() {
  if (current_offset_ >= length_) throw std::out_of_range("Tried to access element past end of buffer.");
  return current_message_;
}

MessageParser::Iterator::pointer MessageParser::Iterator::operator->() {
  if (current_offset_ >= length_) throw std::out_of_range("Tried to access element past end of buffer.");
  return &current_message_;
}

MessageParser::Iterator &MessageParser::Iterator::operator++() {
  if (current_offset_ >= length_) return *this;

  current_offset_ += current_message_.getHeader().length + sizeof(Header);
  if (current_offset_ >= length_) {
    invalidate();
  } else {
    data_ = data_begin_ + current_offset_;
    current_message_ = Message(data_, length_ - current_offset_);
  }

  return *this;
}

bool MessageParser::Iterator::operator==(const MessageParser::Iterator &rhs) const {
  return current_offset_ == rhs.current_offset_&&
      data_ == rhs.data_ &&
      data_begin_ == rhs.data_begin_ &&
      length_ == rhs.length_;
}

bool MessageParser::Iterator::operator!=(const MessageParser::Iterator &rhs) const {
  return !(*this == rhs);
}

MessageParser::Iterator& MessageParser::Iterator::invalidate() {
  current_offset_ = length_;
  data_ = nullptr;
  return *this;
}

}  // namespace applanix_driver::gsof
