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

#include <applanix_msgs/srv/set_origin.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "applanix_driver/gsof/gsof.h"
#include "applanix_driver/lvx_client.h"
#include "util/ros_time_source.h"
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>


namespace applanix_driver_ros {

class LvxClientRos : public rclcpp::Node {
 public:
  struct connection_error : public std::runtime_error {
    explicit connection_error(const std::string &msg = "") :
        std::runtime_error("LvxClientRos could not start a connection to LVX unit. " + msg) {}
  };

  explicit LvxClientRos(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  static constexpr char k_default_node_name[] = "lvx_client";

  static constexpr char k_topic_orientation[]    = "/applanix/lvx_client/autoware_orientation";
  static constexpr char k_topic_odometry[]       = "/applanix/lvx_client/odom";
  static constexpr char k_topic_navsat[]         = "/applanix/lvx_client/gnss/fix";
  static constexpr char k_topic_imu[]            = "/applanix/lvx_client/imu_raw";
  static constexpr char k_topic_twist[]          = "/applanix/lvx_client/twist_with_covariance";
  static constexpr char k_topic_gsof_49[]        = "/applanix/lvx_client/gsof/ins_solution_49";
  static constexpr char k_topic_gsof_50[]        = "/applanix/lvx_client/gsof/ins_solution_rms_50";
  static constexpr char k_service_set_origin[]   = "/lvx_client/set_origin";
  static constexpr char k_service_reset_origin[] = "/lvx_client/reset_origin";



  static constexpr char k_default_parent_frame[] = "ned";
  static constexpr char k_default_child_frame[] = "POS_REF";
  static constexpr char k_default_time_source[] = "now";
  static constexpr char k_default_base_frame_id[] = "base_link";
  static constexpr char k_default_gnss_ins_frame_id[] = "gnss_ins_link";

  static constexpr int k_default_qos_history_depth = 100;

  std::string parent_frame_id_;
  std::string child_frame_id_;
  std::string base_frame_id_;
  std::string gnss_ins_frame_id_;

  std::optional<applanix_driver::LvxClient> lvx_client_;
  std::optional<GeographicLib::LocalCartesian> local_cartesian_;

  util::RosTimeSource time_source_;
  rclcpp::Clock ros_clock_;
  bool publish_gsof_msgs_;
  bool publish_ros_msgs_;
  bool publish_autoware_msgs_;
  bool publish_twist_msgs_;
  bool publish_tf_;


  std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> publishers_;
  std::vector<std::shared_ptr<rclcpp::ServiceBase>> services_;
  std::optional<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::optional<applanix_driver::gsof::InsSolution> ins_solution_;
  std::optional<applanix_driver::gsof::InsSolutionRms> ins_solution_rms_;

  using MessageCallback = void (LvxClientRos::*)(const applanix_driver::gsof::Message &);
  template<typename RosMessageType>
  void registerAndAdvertise(applanix_driver::gsof::Id id, MessageCallback callback, const std::string &topic);
  void registerCallback(applanix_driver::gsof::Id id, MessageCallback callback);
  template <typename RosMessageType>
  void advertise(const std::string &topic);
  template<typename ServiceType>
  using ServiceCallback = void (LvxClientRos::*)(const std::shared_ptr<rmw_request_id_t>,
                                                  const typename ServiceType::Request::SharedPtr,
                                                  typename ServiceType::Response::SharedPtr);
  template<typename ServiceType>
  void createService(const std::string &service_name, ServiceCallback<ServiceType> service_callback);


  // Callbacks who's only job is to save data in private members
  void saveGsof49Callback(const applanix_driver::gsof::Message &message);
  void saveGsof50Callback(const applanix_driver::gsof::Message &message);

  // Callbacks to publish data in "standard" ROS messages
  void publishInsSolutionCallback(const applanix_driver::gsof::Message &message);
  void publishImuMsgCallback(const applanix_driver::gsof::Message &message);

  // Callbacks to publish data in custom Trimble gsof ROS messages
  void publishGsof49Callback(const applanix_driver::gsof::Message &message);
  void publishGsof50Callback(const applanix_driver::gsof::Message &message);


  void publishGnssInsOrientationCallback(const applanix_driver::gsof::Message &message);

  void publishGnssInsTwistCallback(const applanix_driver::gsof::Message &message);

  rclcpp::Time getRosTimestamp(const applanix_driver::gsof::GpsTime& gps_time);

  // Service callbacks
  void setOriginCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const applanix_msgs::srv::SetOrigin::Request::SharedPtr request,
                         applanix_msgs::srv::SetOrigin::Response::SharedPtr response);

  void resetOriginCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std_srvs::srv::Empty::Request::SharedPtr request,
                           std_srvs::srv::Empty::Response::SharedPtr response);
};

}  // namespace applanix_driver_ros
