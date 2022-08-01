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

#include "optional_common.h"

#include <stdexcept>
#include <type_traits>

#if __cplusplus == 201103
#define WISE_ENUM_CONSTEXPR_14
#else
#define WISE_ENUM_CONSTEXPR_14 constexpr
#endif

namespace wise_enum {

/* A simple, *forward* compatible optional implementation. That is, it does not
 * provide the full std::optional interface, but all the interface it does
 * provide is found on std::optional, so it should not be a breaking change to
 * upgrade to std::optional.
 */
template <class T>
class optional {
public:
  static_assert(std::is_enum<T>::value,
                "wise enum optional is only for enum types");
  optional() = default;
  optional(T t) : m_t(t), m_active(true) {}

  WISE_ENUM_CONSTEXPR_14 T &operator*() & { return m_t; }
  constexpr const T &operator*() const & { return m_t; }
  WISE_ENUM_CONSTEXPR_14 T &&operator*() && { return m_t; }
  constexpr const T &&operator*() const && { return m_t; }

  constexpr explicit operator bool() const noexcept { return m_active; }
  constexpr bool has_value() const noexcept { return m_active; }

  WISE_ENUM_CONSTEXPR_14 T &value() & {
    if (m_active)
      return m_t;
    else
      WISE_ENUM_OPTIONAL_BAD_ACCESS;
  }
  WISE_ENUM_CONSTEXPR_14 const T &value() const & {
    if (m_active)
      return m_t;
    else
      WISE_ENUM_OPTIONAL_BAD_ACCESS;
  }

  WISE_ENUM_CONSTEXPR_14 T &&value() && {
    if (m_active)
      return m_t;
    else
      WISE_ENUM_OPTIONAL_BAD_ACCESS;
  }
  WISE_ENUM_CONSTEXPR_14 const T &&value() const && {
    if (m_active)
      return m_t;
    else
      WISE_ENUM_OPTIONAL_BAD_ACCESS;
  }

  template <class U>
  WISE_ENUM_CONSTEXPR_14 T value_or(U &&u) {
    if (m_active)
      return m_t;
    else
      return std::forward<U>(u);
  }

  void reset() noexcept { m_active = false; }

  optional(const optional &other) = default;
  optional(optional &&other) = default;
  optional &operator=(const optional &other) = default;
  optional &operator=(optional &&other) = default;

private:
  T m_t;
  bool m_active = false;
};
} // namespace wise_enum
