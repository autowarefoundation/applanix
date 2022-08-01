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

#ifdef WISE_ENUM_NO_EXCEPT

#include <cstdlib>
#define WISE_ENUM_OPTIONAL_BAD_ACCESS std::abort()

#else

#include <stdexcept>

namespace wise_enum {

struct bad_optional_access : std::exception {
  const char *what() const noexcept override {
    return "Error, attempt to access valueless optional!";
  }
};
} // namespace wise_enum

#define WISE_ENUM_OPTIONAL_BAD_ACCESS throw bad_optional_access{}

#endif
