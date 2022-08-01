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

#include <byteswap.h>   // UNIX Specific
#include <cstring>

namespace applanix_driver::utils {
/**
 * https://stackoverflow.com/questions/3529263/template-specialization-according-to-sizeof-type
 * Problem as old as the internet :-)
 * The Trimble GSOF documentation pages all show sample code to parse data types from big to little endian with
 * verbose pointer incrementation and reassignment. Modern processors usually have single instructions to swap the
 * bytes of a chunk of memory. Proof https://godbolt.org/z/EG7G5U
 */

namespace impl {
template<typename T, std::size_t n>
struct Byteswap;

template<typename T>
struct Byteswap<T, 2> {
  T operator()(T val) const {
    uint16_t bytes;
    memcpy(&bytes, &val, sizeof(bytes));
    bytes = bswap_16(bytes);
    T out_val;
    memcpy(&out_val, &bytes, sizeof(T));
    return out_val;
  }
};

template<typename T>
struct Byteswap<T, 4> {
  T operator()(T val) const {
    uint32_t bytes;
    memcpy(&bytes, &val, sizeof(bytes));
    bytes = bswap_32(bytes);
    T out_val;
    memcpy(&out_val, &bytes, sizeof(T));
    return out_val;
  }
};

template<typename T>
struct Byteswap<T, 8> {
  T operator()(T val) const {
    uint64_t bytes;
    memcpy(&bytes, &val, sizeof(bytes));
    bytes = bswap_64(bytes);
    T out_val;
    memcpy(&out_val, &bytes, sizeof(T));
    return out_val;
  }
};

template<typename T, std::size_t n>
struct ByteswapInPlace;

template<typename T>
struct ByteswapInPlace<T, 2> {
  void operator()(T *val) const {
    uint16_t bytes;
    memcpy(&bytes, val, sizeof(bytes));
    bytes = bswap_16(bytes);
    memcpy(val, &bytes, sizeof(T));
  }
};

template<typename T>
struct ByteswapInPlace<T, 4> {
  void operator()(T *val) const {
    uint32_t bytes;
    memcpy(&bytes, val, sizeof(bytes));
    bytes = bswap_32(bytes);
    memcpy(val, &bytes, sizeof(T));
  }
};

template<typename T>
struct ByteswapInPlace<T, 8> {
  void operator()(T *val) const {
    uint64_t bytes;
    memcpy(&bytes, val, sizeof(bytes));
    bytes = bswap_64(bytes);
    memcpy(val, &bytes, sizeof(T));
  }
};

}  // namespace impl

template<typename T>
T byteswap(T val) {
  return impl::Byteswap<T, sizeof(T)>()(val);
}

template<typename T>
void byteswapInPlace(T *val) {
  impl::ByteswapInPlace<T, sizeof(T)>()(val);
}

}  // namespace applanix_driver::utils
