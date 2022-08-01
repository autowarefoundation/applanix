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

#include <cmath>

#include "applanix_driver/utils/byteswap.h"

using applanix_driver::utils::byteswapInPlace;

namespace applanix_driver {

template<typename T>
static T deg2rad(const T val) {
  static constexpr double DEG2RAD = M_PI / 180.0;
  return static_cast<double>(val) * DEG2RAD;
}

template<typename T>
static T rad2deg(const T val) {
  static constexpr double RAD2DEG = 180.0 / M_PI;
  return static_cast<double>(val) * RAD2DEG;
}

#pragma pack(push, 1)

// Forward declaration so Ned can convert to Enu and vice-versa
template<typename T>
struct Enu;

template<typename T>
struct Ned {
  T north;
  T east;
  T down;

  Ned() = default;
  explicit Ned(const Enu<T>& enu) { fromEnu(enu); }
  void fromEnu(const Enu<T>& enu) {
    north = enu.north;
    east = enu.east;
    down = -enu.up;
  }
  void switchEndianess() {
    byteswapInPlace(&north);
    byteswapInPlace(&east);
    byteswapInPlace(&down);
  }
};

using Nedf = Ned<float>;
using Nedd = Ned<double>;

template<typename T>
struct Enu {
  T east;
  T north;
  T up;

  Enu() = default;
  explicit Enu(const Ned<T>& ned) { fromNed(ned); }
  void fromNed(const Ned<T>& ned) {
    east = ned.east;
    north = ned.north;
    up = -ned.down;
  }
  void switchEndianess() {
    byteswapInPlace(&east);
    byteswapInPlace(&north);
    byteswapInPlace(&up);
  }
};

using Enuf = Enu<float>;
using Enud = Enu<double>;

template<typename T>
struct Rph {
  T roll;
  T pitch;
  T heading;
  void switchEndianess() {
    byteswapInPlace(&roll);
    byteswapInPlace(&pitch);
    byteswapInPlace(&heading);
  }
};

using Rphf = Rph<float>;
using Rphd = Rph<double>;

template<typename T>
struct Xyz {
  T x;
  T y;
  T z;
  void switchEndianess() {
    byteswapInPlace(&x);
    byteswapInPlace(&y);
    byteswapInPlace(&z);
  }
};

using Xyzf = Xyz<float>;
using Xyzd = Xyz<double>;
using Xyzl = Xyz<long>;

template<typename T>
struct Lla {
  T latitude;
  T longitude;
  T altitude;
  void switchEndianess() {
    byteswapInPlace(&latitude);
    byteswapInPlace(&longitude);
    byteswapInPlace(&altitude);
  }
};

using Llaf = Lla<float>;
using Llad = Lla<double>;

#pragma pack(pop)

}  // namespace applanix_driver
