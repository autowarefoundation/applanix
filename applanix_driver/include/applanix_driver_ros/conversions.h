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

#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "applanix_driver/gsof/message.h"
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>


namespace applanix_driver_ros {

geometry_msgs::msg::TransformStamped toTransformStamped(const std_msgs::msg::Header &header,
                                                        const decltype(geometry_msgs::msg::TransformStamped::child_frame_id) &child_frame_id,
                                                        const geometry_msgs::msg::Pose &pose);


nav_msgs::msg::Odometry toOdometry(const applanix_driver::gsof::InsSolution &ins_solution,
                                   const GeographicLib::LocalCartesian &local_cartesian);
nav_msgs::msg::Odometry toOdometry(const applanix_driver::gsof::InsSolution &ins_solution,
                                   const GeographicLib::LocalCartesian &local_cartesian,
                                   const applanix_driver::gsof::InsSolutionRms &ins_solution_rms);

sensor_msgs::msg::NavSatFix toNavSatFix(const applanix_driver::gsof::InsSolution &ins_solution);
sensor_msgs::msg::NavSatFix toNavSatFix(const applanix_driver::gsof::InsSolution &ins_solution,
                                        const applanix_driver::gsof::InsSolutionRms &covariance);

sensor_msgs::msg::Imu toImuMsg(const applanix_driver::gsof::InsSolution &ins_solution);
sensor_msgs::msg::Imu toImuMsg(const applanix_driver::gsof::InsSolution &ins_solution,
                                     const applanix_driver::gsof::InsSolutionRms &covariance);

autoware_sensing_msgs::msg::GnssInsOrientationStamped toAutowareOrientationMsg(const applanix_driver::gsof::InsSolution &ins_solution,
                                                                 const applanix_driver::gsof::InsSolutionRms &covariance);
autoware_sensing_msgs::msg::GnssInsOrientationStamped toAutowareOrientationMsg(const applanix_driver::gsof::InsSolution &ins_solution);

geometry_msgs::msg::TwistWithCovarianceStamped toTwistMsg(const applanix_driver::gsof::InsSolution &ins_solution);

applanix_msgs::msg::GpsTimeGsof toRosMessage(const applanix_driver::gsof::GpsTime &gps_time);
applanix_msgs::msg::StatusGsof toRosMessage(const applanix_driver::gsof::Status &status);
applanix_msgs::msg::NavigationSolutionGsof49 toRosMessage(const applanix_driver::gsof::InsSolution &ins_solution);
applanix_msgs::msg::NavigationPerformanceGsof50 toRosMessage(const applanix_driver::gsof::InsSolutionRms &ins_solution_rms);

/**
 * Convert GpsTime to rclcpp::Time
 */
rclcpp::Time toRosTimeOfTheWeek(const applanix_driver::gsof::GpsTime &gps_time);
rclcpp::Time toRosTimeGpsEpoch(const applanix_driver::gsof::GpsTime &gps_time);


template<typename T>
applanix_msgs::msg::LLA toRosMessage(const applanix_driver::Lla<T> &lla) {
  applanix_msgs::msg::LLA ros_lla;
  ros_lla.latitude = lla.latitude;
  ros_lla.longitude = lla.longitude;
  ros_lla.altitude = lla.altitude;
  return ros_lla;
}

template<typename T>
applanix_msgs::msg::NED toRosMessage(const applanix_driver::Ned<T> &ned) {
  applanix_msgs::msg::NED ros_ned;
  ros_ned.north = ned.north;
  ros_ned.east = ned.east;
  ros_ned.down = ned.down;
  return ros_ned;
}

template<typename T>
geometry_msgs::msg::Point toPoint(const applanix_driver::Xyz<T> &xyz) {
  geometry_msgs::msg::Point p;
  p.x = xyz.x;
  p.y = xyz.y;
  p.z = xyz.z;
  return p;
}

template<typename T>
geometry_msgs::msg::Point toPoint(const applanix_driver::Ned<T> &ned) {
  geometry_msgs::msg::Point p;
  p.x = ned.north;
  p.y = ned.east;
  p.z = ned.down;
  return p;
}

template<typename T>
geometry_msgs::msg::Point toPoint(const applanix_driver::Enu<T> &enu) {
  geometry_msgs::msg::Point p;
  p.x = enu.east;
  p.y = enu.north;
  p.z = enu.up;
  return p;
}

template<typename T>
geometry_msgs::msg::Vector3 toVector(const applanix_driver::Xyz<T> &xyz) {
  geometry_msgs::msg::Vector3 v;
  v.x = xyz.x;
  v.y = xyz.y;
  v.z = xyz.z;
  return v;
}

template<typename T>
geometry_msgs::msg::Vector3 toVector(const applanix_driver::Ned<T> &ned) {
  geometry_msgs::msg::Vector3 v;
  v.x = ned.north;
  v.y = ned.east;
  v.z = ned.down;
  return v;
}

template<typename T>
geometry_msgs::msg::Vector3 toVector(const applanix_driver::Enu<T> &enu) {
  geometry_msgs::msg::Vector3 v;
  v.x = enu.east;
  v.y = enu.north;
  v.z = enu.up;
  return v;
}

template<typename Scalar>
geometry_msgs::msg::Vector3 toVector(const applanix_driver::Rph<Scalar> &rph) {
  geometry_msgs::msg::Vector3 v;
  v.x = rph.roll;
  v.y = rph.pitch;
  v.z = rph.heading;
  return v;
}

}  // namespace applanix_driver_ros
