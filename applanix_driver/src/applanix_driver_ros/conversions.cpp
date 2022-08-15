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

#include "applanix_driver_ros/conversions.h"

#include <GeographicLib/UTMUPS.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "applanix_driver/math.h"
#include <sensor_msgs/msg/imu.hpp>

namespace applanix_driver_ros
{

using applanix_driver::deg2rad;

geometry_msgs::msg::TransformStamped toTransformStamped(
  const std_msgs::msg::Header & header,
  const decltype(geometry_msgs::msg::TransformStamped::child_frame_id) & child_frame_id,
  const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::TransformStamped t;
  t.header = header;
  t.child_frame_id = child_frame_id;
  t.transform.translation.x = pose.position.x;
  t.transform.translation.y = pose.position.y;
  t.transform.translation.z = pose.position.z;
  t.transform.rotation = pose.orientation;
  return t;
}

constexpr std::int64_t k_to_nano = 1e9;
constexpr std::int64_t k_milli_to_nano = 1e6;
constexpr std::int64_t k_gps_seconds_in_week = 60 * 60 * 24 * 7;

rclcpp::Time toRosTimeOfTheWeek(const applanix_driver::gsof::GpsTime & gps_time)
{
  return rclcpp::Time(gps_time.time_msec * k_milli_to_nano, RCL_STEADY_TIME);
}

rclcpp::Time toRosTimeGpsEpoch(const applanix_driver::gsof::GpsTime & gps_time)
{
  // GpsTime has week as 16 bit so we don't need to take week rollover into account
  const std::int64_t gps_nanoseconds_since_epoch =
    (k_gps_seconds_in_week * static_cast<std::int64_t>(gps_time.week) * k_to_nano) +
    (k_milli_to_nano * gps_time.time_msec);
  return rclcpp::Time(gps_nanoseconds_since_epoch, RCL_STEADY_TIME);
}

nav_msgs::msg::Odometry toOdometry(
  const applanix_driver::gsof::InsSolution & ins_solution,
  const GeographicLib::LocalCartesian & local_cartesian)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = toRosTimeOfTheWeek(ins_solution.gps_time);

  applanix_driver::Enud enu;
  const auto & lla = ins_solution.lla;
  local_cartesian.Forward(lla.latitude, lla.longitude, lla.altitude, enu.east, enu.north, enu.up);

  // Convert to NED while we're at it
  applanix_driver::Nedd ned(enu);
  odom.pose.pose.position = toPoint(ned);

  tf2::Quaternion q;
  q.setEuler(
    deg2rad(ins_solution.attitude.heading),
    deg2rad(ins_solution.attitude.pitch),
    deg2rad(ins_solution.attitude.roll));
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear = toVector(ins_solution.velocity);
  odom.twist.twist.angular = toVector(ins_solution.angular_rate);

  odom.twist.twist.angular.x = deg2rad(odom.twist.twist.angular.x);
  odom.twist.twist.angular.y = deg2rad(odom.twist.twist.angular.y);
  odom.twist.twist.angular.z = deg2rad(odom.twist.twist.angular.z);

  odom.pose.covariance[0] = -1;
  odom.twist.covariance[0] = -1;

  return odom;
}

nav_msgs::msg::Odometry toOdometry(
  const applanix_driver::gsof::InsSolution & ins_solution,
  const GeographicLib::LocalCartesian & local_cartesian,
  const applanix_driver::gsof::InsSolutionRms & ins_solution_rms)
{
  nav_msgs::msg::Odometry odom = toOdometry(ins_solution, local_cartesian);

  odom.pose.covariance[0] = std::pow(ins_solution_rms.position_rms.north, 2);
  odom.pose.covariance[7] = std::pow(ins_solution_rms.position_rms.east, 2);
  odom.pose.covariance[14] = std::pow(ins_solution_rms.position_rms.down, 2);
  odom.pose.covariance[21] = std::pow(deg2rad(ins_solution_rms.attitude_rms.roll), 2);
  odom.pose.covariance[28] = std::pow(deg2rad(ins_solution_rms.attitude_rms.pitch), 2);
  odom.pose.covariance[35] = std::pow(deg2rad(ins_solution_rms.attitude_rms.heading), 2);

  odom.twist.covariance[0] = std::pow(ins_solution_rms.velocity_rms.north, 2);
  odom.twist.covariance[7] = std::pow(ins_solution_rms.velocity_rms.east, 2);
  odom.twist.covariance[14] = std::pow(ins_solution_rms.velocity_rms.down, 2);

  return odom;
}

sensor_msgs::msg::NavSatFix toNavSatFix(const applanix_driver::gsof::InsSolution & ins_solution)
{
  sensor_msgs::msg::NavSatFix nav_sat_fix;
  nav_sat_fix.latitude = ins_solution.lla.latitude;
  nav_sat_fix.longitude = ins_solution.lla.longitude;
  nav_sat_fix.altitude = ins_solution.lla.altitude;

  nav_sat_fix.position_covariance[0] = -1;  // Means unknown

  using GnssStatus = applanix_driver::gsof::Status::GnssStatus;

  switch (ins_solution.status.getGnssStatus()) {
    case GnssStatus::GNSS_SPS_MODE:
    case GnssStatus::GPS_PPS_MODE:
    case GnssStatus::DIRECT_GEOREFERENCING_MODE:
    case GnssStatus::FLOAT_RTK:
      nav_sat_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case GnssStatus::DIFFERENTIAL_GPS_SPS:
    case GnssStatus::FIXED_RTK_MODE:
      nav_sat_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    case GnssStatus::FIX_NOT_AVAILABLE:
    case GnssStatus::UNKNOWN:
      nav_sat_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
  }

  // Note(Andre) There are 3 GSOF messages giving more details about which space vehicles are being used, we can use
  //  those to populate this field if we really need it. However, the enumerations in the sensor_msgs package don't
  //  cover nearly as many constellations Trimble products can receive from.
  nav_sat_fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  return nav_sat_fix;
}

sensor_msgs::msg::NavSatFix toNavSatFix(
  const applanix_driver::gsof::InsSolution & ins_solution,
  const applanix_driver::gsof::InsSolutionRms & covariance)
{
  sensor_msgs::msg::NavSatFix nav_sat_fix = toNavSatFix(ins_solution);

  // XXX ROS NavSatFix says position covariance is in ENU
  nav_sat_fix.position_covariance[0] = std::pow(covariance.position_rms.east, 2);
  nav_sat_fix.position_covariance[4] = std::pow(covariance.position_rms.north, 2);
  nav_sat_fix.position_covariance[8] = std::pow(covariance.position_rms.down, 2);

  nav_sat_fix.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return nav_sat_fix;
}

sensor_msgs::msg::Imu toImuMsg(const applanix_driver::gsof::InsSolution & ins_solution)
{
    sensor_msgs::msg::Imu imuMsg;

    tf2::Quaternion quaternion, ins_corrected_quat;
    quaternion.setRPY(
            deg2rad(ins_solution.attitude.roll),
            deg2rad(ins_solution.attitude.pitch),
            deg2rad(ins_solution.attitude.heading)
            );

    tf2::Matrix3x3 ENU2NED, ins_rot_matrix, ins_rot_, ins_corrected_rot_, applanix2ros;
    ins_rot_matrix.setRotation(quaternion);

    ENU2NED = tf2::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1);
    applanix2ros = tf2::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    ins_rot_ = ENU2NED * ins_rot_matrix;

    ins_corrected_rot_ = ins_rot_ * applanix2ros;
    ins_corrected_rot_.getRotation(ins_corrected_quat);

    imuMsg.orientation.x = ins_corrected_quat.getX();
    imuMsg.orientation.y = ins_corrected_quat.getY();
    imuMsg.orientation.z = ins_corrected_quat.getZ();
    imuMsg.orientation.w = ins_corrected_quat.getW();

    imuMsg.angular_velocity.x = static_cast<double>(deg2rad(ins_solution.angular_rate.roll));
    imuMsg.angular_velocity.y = static_cast<double>(-deg2rad(ins_solution.angular_rate.pitch));
    imuMsg.angular_velocity.z = static_cast<double>(-deg2rad(ins_solution.angular_rate.heading));


    imuMsg.linear_acceleration.x = static_cast<double>(ins_solution.acceleration.x);
    imuMsg.linear_acceleration.y = static_cast<double>(-ins_solution.acceleration.y);
    imuMsg.linear_acceleration.z = static_cast<double>(-ins_solution.acceleration.z);

    return imuMsg;

}

sensor_msgs::msg::Imu toImuMsg(
            const applanix_driver::gsof::InsSolution & ins_solution,
            const applanix_driver::gsof::InsSolutionRms & covariance)
{
    sensor_msgs::msg::Imu imuMsg = toImuMsg(ins_solution);

    imuMsg.orientation_covariance[0] = std::pow(deg2rad(covariance.attitude_rms.roll), 2);
    imuMsg.orientation_covariance[4] = std::pow(deg2rad(covariance.attitude_rms.pitch), 2);
    imuMsg.orientation_covariance[8] = std::pow(deg2rad(covariance.attitude_rms.heading), 2);

    imuMsg.angular_velocity_covariance[0] = 0.1;
    imuMsg.angular_velocity_covariance[4] = 0.1;
    imuMsg.angular_velocity_covariance[8] = 10;

    imuMsg.linear_acceleration_covariance[0] = 0.1;
    imuMsg.linear_acceleration_covariance[4] = 0.1;
    imuMsg.linear_acceleration_covariance[8] = 0.1;

    return imuMsg;
}

autoware_sensing_msgs::msg::GnssInsOrientationStamped toAutowareOrientationMsg(
            const applanix_driver::gsof::InsSolution & ins_solution)
{

    autoware_sensing_msgs::msg::GnssInsOrientationStamped autowareOrientationMsg;

    tf2::Quaternion quaternion, ins_corrected_quat;
    quaternion.setRPY(
            deg2rad(ins_solution.attitude.roll),
            deg2rad(ins_solution.attitude.pitch),
            deg2rad(ins_solution.attitude.heading)
    );

    tf2::Matrix3x3 ENU2NED, ins_rot_matrix, ins_rot_, ins_corrected_rot_, applanix2ros;
    ins_rot_matrix.setRotation(quaternion);

    ENU2NED = tf2::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1);
    applanix2ros = tf2::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    ins_rot_ = ENU2NED * ins_rot_matrix;

    ins_corrected_rot_ = ins_rot_ * applanix2ros;
    ins_corrected_rot_.getRotation(ins_corrected_quat);

    autowareOrientationMsg.orientation.orientation.x = ins_corrected_quat.getX();
    autowareOrientationMsg.orientation.orientation.y = ins_corrected_quat.getY();
    autowareOrientationMsg.orientation.orientation.z = ins_corrected_quat.getZ();
    autowareOrientationMsg.orientation.orientation.w = ins_corrected_quat.getW();

    return autowareOrientationMsg;
}

autoware_sensing_msgs::msg::GnssInsOrientationStamped toAutowareOrientationMsg(
            const applanix_driver::gsof::InsSolution & ins_solution,
            const applanix_driver::gsof::InsSolutionRms & covariance)
{

    autoware_sensing_msgs::msg::GnssInsOrientationStamped autowareOrientationMsg = toAutowareOrientationMsg(ins_solution);

    autowareOrientationMsg.orientation.rmse_rotation_x = covariance.attitude_rms.roll;
    autowareOrientationMsg.orientation.rmse_rotation_y = covariance.attitude_rms.pitch;
    autowareOrientationMsg.orientation.rmse_rotation_z = covariance.attitude_rms.heading;

    return autowareOrientationMsg;
}

geometry_msgs::msg::TwistWithCovarianceStamped toTwistMsg(
           const applanix_driver::gsof::InsSolution & ins_solution)
{

    geometry_msgs::msg::TwistWithCovarianceStamped twistMsg;

    if(cos(deg2rad(ins_solution.attitude.heading - ins_solution.track_angle)) < 0) {
        twistMsg.twist.twist.linear.x = - ins_solution.total_speed;
    }
    else {
        twistMsg.twist.twist.linear.x = ins_solution.total_speed;
    }

    twistMsg.twist.twist.angular.z = - deg2rad(ins_solution.angular_rate.heading);
    twistMsg.twist.covariance[0]  = 0.04;
    twistMsg.twist.covariance[7]  = 10000.0;
    twistMsg.twist.covariance[14] = 10000.0;
    twistMsg.twist.covariance[21] = 10000.0;
    twistMsg.twist.covariance[28] = 10000.0;
    twistMsg.twist.covariance[35] = 0.01;

   return twistMsg;
}

applanix_msgs::msg::GpsTimeGsof toRosMessage(const applanix_driver::gsof::GpsTime & gps_time)
{
  applanix_msgs::msg::GpsTimeGsof gps_time_gsof;
  gps_time_gsof.time = gps_time.time_msec;
  gps_time_gsof.week = gps_time.week;
  return gps_time_gsof;
}

applanix_msgs::msg::StatusGsof toRosMessage(const applanix_driver::gsof::Status & status)
{
  applanix_msgs::msg::StatusGsof status_gsof;
  status_gsof.gnss = status.gnss;
  status_gsof.imu_alignment = status.imu_alignment;
  return status_gsof;
}

applanix_msgs::msg::NavigationSolutionGsof49 toRosMessage(
  const applanix_driver::gsof::InsSolution & ins_solution)
{
  applanix_msgs::msg::NavigationSolutionGsof49 sol;
  sol.header.stamp = toRosTimeOfTheWeek(ins_solution.gps_time);
  sol.gps_time = toRosMessage(ins_solution.gps_time);
  sol.status = toRosMessage(ins_solution.status);
  sol.lla = toRosMessage(ins_solution.lla);
  sol.velocity = toRosMessage(ins_solution.velocity);
  sol.total_speed = ins_solution.total_speed;
  sol.roll = ins_solution.attitude.roll;
  sol.pitch = ins_solution.attitude.pitch;
  sol.heading = ins_solution.attitude.heading;
  sol.track_angle = ins_solution.track_angle;
  sol.ang_rate_long = ins_solution.angular_rate.roll;
  sol.ang_rate_trans = ins_solution.angular_rate.pitch;
  sol.ang_rate_down = ins_solution.angular_rate.heading;
  sol.acc_long = ins_solution.acceleration.x;
  sol.acc_trans = ins_solution.acceleration.y;
  sol.acc_down = ins_solution.acceleration.z;

  return sol;
}

applanix_msgs::msg::NavigationPerformanceGsof50 toRosMessage(
  const applanix_driver::gsof::InsSolutionRms & ins_solution_rms)
{
  applanix_msgs::msg::NavigationPerformanceGsof50 ros_rms;
  ros_rms.header.stamp = toRosTimeOfTheWeek(ins_solution_rms.gps_time);
  ros_rms.gps_time = toRosMessage(ins_solution_rms.gps_time);
  ros_rms.status = toRosMessage(ins_solution_rms.status);
  ros_rms.pos_rms_error = toRosMessage(ins_solution_rms.position_rms);
  ros_rms.vel_rms_error = toRosMessage(ins_solution_rms.velocity_rms);
  ros_rms.attitude_rms_error_roll = ins_solution_rms.attitude_rms.roll;
  ros_rms.attitude_rms_error_pitch = ins_solution_rms.attitude_rms.pitch;
  ros_rms.attitude_rms_error_heading = ins_solution_rms.attitude_rms.heading;
  return ros_rms;
}

}  // namespace applanix_driver_ros
