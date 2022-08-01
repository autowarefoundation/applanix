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
#include <cstddef>
#include <type_traits>

// Contains definitions for LLa<T>, Ned<T>, Rph<T>, Xyz<T>
#include "applanix_driver/math.h"

namespace applanix_driver::gsof {

using Id = std::uint8_t;
static constexpr Id GSOF_ID_49_INS_FULL_NAV = 49U;
static constexpr Id GSOF_ID_50_INS_RMS = 50U;

/**
 * A lot of the internal classes for message parsing have the void switchEndianess() member function. While it seems
 * like it should be implementing an interface, this would create dynamic objects with a vtable which is not
 * compatible with memcpy for type punning.
 */

#pragma pack(push, 1)
struct Header {
  Id type;
  uint8_t length;
};

struct GpsTime {
  uint16_t week;
  uint32_t time_msec;
  void switchEndianess() {
    byteswapInPlace(&week);
    byteswapInPlace(&time_msec);
  }
};

struct Status {
  uint8_t imu_alignment;
  uint8_t gnss;

  enum class ImuAlignmentStatus {
    GPS_ONLY,
    COARSE_LEVELING,
    DEGRADED,
    ALIGNED,
    FULL_NAV,
    UNKNOWN
  };

  [[nodiscard]] ImuAlignmentStatus getImuAlignmentStatus() const {
    switch (imu_alignment) {
      case 0:
        return ImuAlignmentStatus::GPS_ONLY;
      case 1:
        return ImuAlignmentStatus::COARSE_LEVELING;
      case 2:
        return ImuAlignmentStatus::DEGRADED;
      case 3:
        return ImuAlignmentStatus::ALIGNED;
      case 4:
        return ImuAlignmentStatus::FULL_NAV;
      default:
        return ImuAlignmentStatus::UNKNOWN;
    }
  }

  enum class GnssStatus {
    FIX_NOT_AVAILABLE,
    GNSS_SPS_MODE,
    DIFFERENTIAL_GPS_SPS,
    GPS_PPS_MODE,
    FIXED_RTK_MODE,
    FLOAT_RTK,
    DIRECT_GEOREFERENCING_MODE,
    UNKNOWN
  };

  [[nodiscard]] GnssStatus getGnssStatus() const {
    switch (gnss) {
      case 0:
        return GnssStatus::FIX_NOT_AVAILABLE;
      case 1:
        return GnssStatus::GNSS_SPS_MODE;
      case 2:
        return GnssStatus::DIFFERENTIAL_GPS_SPS;
      case 3:
        return GnssStatus::GPS_PPS_MODE;
      case 4:
        return GnssStatus::FIXED_RTK_MODE;
      case 5:
        return GnssStatus::FLOAT_RTK;
      case 6:
        return GnssStatus::DIRECT_GEOREFERENCING_MODE;
      default:
        return GnssStatus::UNKNOWN;
    }
  }
};

/* GSOF #49 : Full Navigation Info
 *
 * 1 (byte)     OUTPUT RECORD TYPE
 * 1 (byte)     RECORD LENGTH
 * 2 (short)    GPS WEEK NUMBER
 * 4 (int)      GPS time in msec of current week
 * 1 (byte)     IMU alignment status
 * 1 (byte)     GPS Quality indicator
 * 8 (double)   Latitude (degrees)
 * 8 (double)   Longitude (degrees)
 * 8 (double)   Altitude (m)
 * 4 (float)    North velocity (m/s)
 * 4 (float)    East velocity  (m/s)
 * 4 (float)    Down velocity  (m/s)
 * 4 (float)    Total speed (m/s)
 * 8 (double)   Roll     (degrees)
 * 8 (double)   Pitch    (degrees)
 * 8 (double)   Heading  (degrees)
 * 8 (double)   Track angle (degrees)
 * 4 (float)    Angular rate about longitudinal axis (X) (deg/sec)
 * 4 (float)    Angular rate about traverse axis (Y) (deg/sec)
 * 4 (float)    Angular rate about down axis (Z) (deg/sec)
 * 4 (float)    Longitudinal accelaration (X) (m/s^2)
 * 4 (float)    Traverse acceleration (Y) (m/s^2)
 * 4 (float)    Down acceleration (Z) (m/s^2)
*/
struct InsSolution {
  Header header;
  GpsTime gps_time;
  Status status;
  Llad lla;                           // [deg, m]
  Nedf velocity;                      // [m / s]
  float total_speed;                  // [m / s]
  Rphd attitude;                      // [deg]
  double track_angle;                 // [deg]
  Rphf angular_rate;                  // [deg / s]
  Xyzf acceleration;                  // [m / s^2]
  void switchEndianess() {
    gps_time.switchEndianess();
    // status already aligned
    lla.switchEndianess();
    velocity.switchEndianess();
    byteswapInPlace(&total_speed);
    attitude.switchEndianess();
    byteswapInPlace(&track_angle);
    angular_rate.switchEndianess();
    acceleration.switchEndianess();
  }
};

/* GSOF #50 : INS RMS Info
 *
 * 1 (byte)     OUTPUT RECORD TYPE
 * 1 (byte)     RECORD LENGTH
 * 2 (short)    GPS WEEK NUMBER
 * 4 (int)      GPS time in msec of current week
 * 1 (byte)     IMU alignment status
 * 1 (byte)     GPS Quality indicator
 * 4 (float)    North Position RMS m
 * 4 (float)    East Position RMS  m
 * 4 (float)    Down Position RMS  m
 * 4 (float)    North Velocity RMS m/s
 * 4 (float)    East Velocity RMS m/s
 * 4 (float)    Down Velocity RMS m/s
 * 4 (float)    Roll RMS (deg)
 * 4 (float)    Pitch RMS (deg)
 * 4 (float)    Heading RMS (deg)
*/
struct InsSolutionRms {
  Header header;
  GpsTime gps_time;
  Status status;
  Nedf position_rms;
  Nedf velocity_rms;
  Rphf attitude_rms;
  void switchEndianess() {
    gps_time.switchEndianess();
    position_rms.switchEndianess();
    velocity_rms.switchEndianess();
    attitude_rms.switchEndianess();
  }
};


#pragma pack(pop)

namespace impl {

// Test whether class T has the function switchEndianess(void), doesn't check the return type.
struct test_has_switch_endianess {
  template<class T>
  static auto test(T *p) -> decltype(p->switchEndianess(), std::true_type());

  template<class>
  static auto test(...) -> std::false_type;
};

}  // namespace impl

// Type trait for test_has_switch_endianess
template<class T>
struct has_switch_endianess : decltype(impl::test_has_switch_endianess::test<T>(0)) {
  static constexpr bool value = decltype(impl::test_has_switch_endianess::test<T>(0))();
};

template<class T>
inline constexpr bool has_switch_endianess_v = has_switch_endianess<T>::value;

class Message {
 public:
  Message(const std::byte *data, std::size_t length);
  [[nodiscard]] Header getHeader() const;

  template<typename T>
  [[nodiscard]] T as() const {
    static_assert(has_switch_endianess_v<T>, "Class has to implement function switchEndianess().");
    static_assert(std::is_standard_layout_v<T>);
    static_assert(std::is_trivial_v<T>);
    T t;
    std::memcpy(&t, data_, sizeof(T));
    t.switchEndianess();
    return t;
  }

 private:
  const std::byte *data_;
  std::size_t length_;
};

}  // namespace applanix_driver::gsof
