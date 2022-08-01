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

#include "util/wise_enum/wise_enum.h"

namespace util {

WISE_ENUM_CLASS(ErrorCode, OK, CANCELLED, INVALID_ARGUMENT, NOT_FOUND, UNIMPLEMENTED,
                CONNECTION_ERROR, DATA_EMPTY)

class Status {
 public:
  explicit Status(ErrorCode code = ErrorCode::OK, const std::string &msg = "");

  explicit operator bool() const;
  bool ok() const;
  ErrorCode error_code() const;
  const std::string &error_msg() const;

  std::string toString() const;

 private:
  ErrorCode error_code_;
  std::string error_msg_;
};

}  // namespace util
