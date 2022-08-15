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

#include "applanix_driver_ros/lvx_client_ros.hpp"

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "applanix_driver_ros/conversions.h"

namespace applanix_driver_ros {

LvxClientRos::LvxClientRos(const rclcpp::NodeOptions &options) :
    Node(k_default_node_name, options),
    parent_frame_id_(),
    child_frame_id_(),
    lvx_client_(std::nullopt),
    local_cartesian_(std::nullopt),
    time_source_(util::RosTimeSource::GPS_TIME_OF_WEEK),
    ros_clock_(RCL_ROS_TIME),
    publish_gsof_msgs_(true),
    publish_ros_msgs_(true),
    publish_autoware_msgs_(true),
    publish_twist_msgs_(true),
    publish_tf_(true),
    publishers_(),
    transform_broadcaster_(std::nullopt) {

  std::string ip = this->declare_parameter("lvx_ip", "192.168.1.100");
  int port = this->declare_parameter("port", 5602);

  lvx_client_.emplace(ip, port);

  parent_frame_id_ = this->declare_parameter("parent_frame", k_default_parent_frame);
  child_frame_id_ = this->declare_parameter("child_frame", k_default_child_frame);
  base_frame_id_ = this->declare_parameter("base_frame",k_default_base_frame_id);
  gnss_ins_frame_id_ = this->declare_parameter("gnss_ins_frame_id",k_default_gnss_ins_frame_id);
  publish_gsof_msgs_ = this->declare_parameter("publish_gsof_msgs", true);
  publish_ros_msgs_ = this->declare_parameter("publish_ros_msgs", true);
  publish_autoware_msgs_ = this->declare_parameter("publish_autoware_msgs",true);
  publish_twist_msgs_ = this->declare_parameter("publish_twist_msgs",true);

  bool publish_tf = this->declare_parameter("publish_tf", true);
  if (publish_tf) {
    transform_broadcaster_.emplace(this);
  }

  std::string time_source = this->declare_parameter("time_source", k_default_time_source);
  time_source_ = util::toRosTimeSource(time_source);

  // Note: Put the save gsof callbacks before the publish callbacks
  registerCallback(applanix_driver::gsof::GSOF_ID_49_INS_FULL_NAV, &LvxClientRos::saveGsof49Callback);
  registerCallback(applanix_driver::gsof::GSOF_ID_50_INS_RMS, &LvxClientRos::saveGsof50Callback);

  if (publish_ros_msgs_) {
    registerAndAdvertise<nav_msgs::msg::Odometry>(applanix_driver::gsof::GSOF_ID_49_INS_FULL_NAV,
                                                  &LvxClientRos::publishInsSolutionCallback,
                                                  k_topic_odometry);
    advertise<sensor_msgs::msg::NavSatFix>(k_topic_navsat);

    registerAndAdvertise<sensor_msgs::msg::Imu>(applanix_driver::gsof::GSOF_ID_49_INS_FULL_NAV,
                                                  &LvxClientRos::publishImuMsgCallback,
                                                  k_topic_imu);
    advertise<sensor_msgs::msg::Imu>(k_topic_imu);
  }

  if (publish_gsof_msgs_) {
    registerAndAdvertise<applanix_msgs::msg::NavigationSolutionGsof49>(applanix_driver::gsof::GSOF_ID_49_INS_FULL_NAV,
                                                                       &LvxClientRos::publishGsof49Callback,
                                                                       k_topic_gsof_49);
    registerAndAdvertise<applanix_msgs::msg::NavigationPerformanceGsof50>(applanix_driver::gsof::GSOF_ID_50_INS_RMS,
                                                                          &LvxClientRos::publishGsof50Callback,
                                                                          k_topic_gsof_50);
  }
  if(publish_autoware_msgs_){

      registerAndAdvertise<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(applanix_driver::gsof::GSOF_ID_49_INS_FULL_NAV,
                                                                               &LvxClientRos::publishGnssInsOrientationCallback,
                                                                               k_topic_orientation);
      advertise<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(k_topic_orientation);
  }
  if(publish_twist_msgs_){

      registerAndAdvertise<geometry_msgs::msg::TwistWithCovarianceStamped>(applanix_driver::gsof::GSOF_ID_49_INS_FULL_NAV,
                                                                                    &LvxClientRos::publishGnssInsTwistCallback,
                                                                                    k_topic_twist);
      advertise<geometry_msgs::msg::TwistWithCovarianceStamped>(k_topic_twist);
  }
  createService<applanix_msgs::srv::SetOrigin>(k_service_set_origin, &LvxClientRos::setOriginCallback);
  createService<std_srvs::srv::Empty>(k_service_reset_origin, &LvxClientRos::resetOriginCallback);

  util::Status status = lvx_client_->start();
  if (!status) {
    RCLCPP_ERROR(this->get_logger(), "Error starting lvx client: %s", status.error_msg().c_str());
    throw connection_error(status.error_msg());
  }

  RCLCPP_INFO(this->get_logger(), "POS LVX client connection started to %s:%d", ip.c_str(), port);
}

template<typename RosMessageType>
void LvxClientRos::registerAndAdvertise(applanix_driver::gsof::Id id,
                                         LvxClientRos::MessageCallback callback,
                                         const std::string &topic) {
  registerCallback(id, callback);
  advertise<RosMessageType>(topic);
}

void LvxClientRos::registerCallback(applanix_driver::gsof::Id id, LvxClientRos::MessageCallback callback) {
  lvx_client_->registerCallback(id, [this, callback](const applanix_driver::gsof::Message &msg) {
    (this->*callback)(msg);
  });
}

template<typename RosMessageType>
void LvxClientRos::advertise(const std::string &topic) {
  publishers_[topic] = this->create_publisher<RosMessageType>(topic, k_default_qos_history_depth);
}

template<typename ServiceType>
void LvxClientRos::createService(const std::string &service_name,
                                  ServiceCallback<ServiceType> service_callback) {
  auto lambda_bound_to_this = [this, service_callback](std::shared_ptr<rmw_request_id_t> request_header,
                                                       typename ServiceType::Request::SharedPtr request,
                                                       typename ServiceType::Response::SharedPtr response) -> void {
    (this->*service_callback)(request_header, request, response);
  };

  typename rclcpp::Service<ServiceType>::SharedPtr
      service = this->create_service<ServiceType>(service_name, lambda_bound_to_this);
  services_.emplace_back(service);  // Should auto case up to std::shared_ptr<rclcpp::ServiceBase>
}

void LvxClientRos::saveGsof49Callback(const applanix_driver::gsof::Message &message) {
  const auto &ins_solution = message.as<applanix_driver::gsof::InsSolution>();
  ins_solution_.emplace(ins_solution);

  if (!local_cartesian_) {
    const auto &lla = ins_solution.lla;
    RCLCPP_INFO(this->get_logger(), "Initializing local cartesian plane to LLA: %4.6f %4.6f %4.6f",
                lla.latitude, lla.longitude, lla.altitude);
    local_cartesian_.emplace(lla.latitude, lla.longitude, lla.altitude, GeographicLib::Geocentric::WGS84());
  }
}

void LvxClientRos::saveGsof50Callback(const applanix_driver::gsof::Message &message) {
  ins_solution_rms_.emplace(message.as<applanix_driver::gsof::InsSolutionRms>());
}

void LvxClientRos::publishGsof49Callback(const applanix_driver::gsof::Message &) {
  using RosMessageType = applanix_msgs::msg::NavigationSolutionGsof49;
  auto publisher = std::static_pointer_cast<rclcpp::Publisher<RosMessageType>>(publishers_[k_topic_gsof_49]);
  publisher->publish(toRosMessage(*ins_solution_));
}

void LvxClientRos::publishGsof50Callback(const applanix_driver::gsof::Message &) {
  using RosMessageType = applanix_msgs::msg::NavigationPerformanceGsof50;
  auto publisher = std::static_pointer_cast<rclcpp::Publisher<RosMessageType>>(publishers_[k_topic_gsof_50]);
  publisher->publish(toRosMessage(*ins_solution_rms_));
}

void LvxClientRos::publishInsSolutionCallback(const applanix_driver::gsof::Message &) {
  sensor_msgs::msg::NavSatFix nav_sat_fix = ins_solution_rms_
                                            ? toNavSatFix(*ins_solution_, *ins_solution_rms_)
                                            : toNavSatFix(*ins_solution_);

  nav_sat_fix.header.frame_id = child_frame_id_;  // Our child frame is the sensor frame
  nav_sat_fix.header.stamp = getRosTimestamp(ins_solution_->gps_time);

  using NavSatFix = sensor_msgs::msg::NavSatFix;
  auto nav_sat_pub = std::static_pointer_cast<rclcpp::Publisher<NavSatFix>>(publishers_[k_topic_navsat]);
  nav_sat_pub->publish(nav_sat_fix);

  nav_msgs::msg::Odometry odom = ins_solution_rms_
                                 ? toOdometry(*ins_solution_, *local_cartesian_, *ins_solution_rms_)
                                 : toOdometry(*ins_solution_, *local_cartesian_);

  odom.child_frame_id = child_frame_id_;
  odom.header.frame_id = parent_frame_id_;
  odom.header.stamp = nav_sat_fix.header.stamp;  // Don't reconvert gps time in case we are using ros time now

  using Odometry = nav_msgs::msg::Odometry;
  auto odom_pub = std::static_pointer_cast<rclcpp::Publisher<Odometry>>(publishers_[k_topic_odometry]);
  odom_pub->publish(odom);

  if (transform_broadcaster_) {
    transform_broadcaster_->sendTransform(toTransformStamped(odom.header, odom.child_frame_id, odom.pose.pose));
  }
}

void LvxClientRos::publishImuMsgCallback(const applanix_driver::gsof::Message &) {
  sensor_msgs::msg::Imu ImuMsg = ins_solution_rms_
                                                  ? toImuMsg(*ins_solution_, *ins_solution_rms_)
                                                  : toImuMsg(*ins_solution_);

  ImuMsg.header.frame_id = base_frame_id_;
  ImuMsg.header.stamp = getRosTimestamp(ins_solution_->gps_time);

  using Imu = sensor_msgs::msg::Imu;
  auto imu_pub = std::static_pointer_cast<rclcpp::Publisher<Imu>>(publishers_[k_topic_imu]);
  imu_pub->publish(ImuMsg);
}

void LvxClientRos::publishGnssInsOrientationCallback(const applanix_driver::gsof::Message &) {

  autoware_sensing_msgs::msg::GnssInsOrientationStamped gnssInsOrientationStamped = ins_solution_rms_
                                                                                      ? toAutowareOrientationMsg(*ins_solution_, *ins_solution_rms_)
                                                                                      : toAutowareOrientationMsg(*ins_solution_);

  gnssInsOrientationStamped.header.frame_id = gnss_ins_frame_id_;
  gnssInsOrientationStamped.header.stamp = getRosTimestamp(ins_solution_->gps_time);

  using GnssInsOrientation = autoware_sensing_msgs::msg::GnssInsOrientationStamped;
  auto autoware_orientation_pub = std::static_pointer_cast<rclcpp::Publisher<GnssInsOrientation>>(publishers_[k_topic_orientation]);
  autoware_orientation_pub->publish(gnssInsOrientationStamped);
}

void LvxClientRos::publishGnssInsTwistCallback(const applanix_driver::gsof::Message &) {

    geometry_msgs::msg::TwistWithCovarianceStamped gnssInsTwistStamped = toTwistMsg(*ins_solution_);

    gnssInsTwistStamped.header.frame_id = gnss_ins_frame_id_;
    gnssInsTwistStamped.header.stamp = getRosTimestamp(ins_solution_->gps_time);

    using GnssInsTwist = geometry_msgs::msg::TwistWithCovarianceStamped;
    auto twist_pub = std::static_pointer_cast<rclcpp::Publisher<GnssInsTwist>>(publishers_[k_topic_twist]);
    twist_pub->publish(gnssInsTwistStamped);
}
rclcpp::Time LvxClientRos::getRosTimestamp(const applanix_driver::gsof::GpsTime &gps_time) {
  switch (time_source_) {
    case util::RosTimeSource::NOW:
      return ros_clock_.now();
    case util::RosTimeSource::GPS_TIME_OF_WEEK:
      return toRosTimeOfTheWeek(gps_time);
    case util::RosTimeSource::GPS:
      return toRosTimeGpsEpoch(gps_time);
    default:
      // Should never happen because we are protected by -Wswitch-enum
      throw std::logic_error("Unhandled RosTimeSource.");
  }
}

void LvxClientRos::setOriginCallback(const std::shared_ptr<rmw_request_id_t>,
                                      const applanix_msgs::srv::SetOrigin::Request::SharedPtr request,
                                      applanix_msgs::srv::SetOrigin::Response::SharedPtr) {
  if (local_cartesian_) {
    local_cartesian_->Reset(request->latitude, request->longitude, request->altitude);
  } else {
    local_cartesian_.emplace(request->latitude, request->longitude, request->altitude);
  }

  RCLCPP_INFO(this->get_logger(),
              "NED origin reset to Lat: %f Lon: %f Alt: %f",
              request->latitude,
              request->longitude,
              request->altitude);
}
void LvxClientRos::resetOriginCallback(const std::shared_ptr<rmw_request_id_t>,
                                        const std_srvs::srv::Empty::Request::SharedPtr,
                                        std_srvs::srv::Empty::Response::SharedPtr) {
  local_cartesian_.reset();
  RCLCPP_INFO(this->get_logger(), "NED origin reset.");
}

}  // namespace applanix_driver_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(applanix_driver_ros::LvxClientRos)
