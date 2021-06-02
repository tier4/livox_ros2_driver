//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#ifndef LIVOX_ROS_DRIVER_LDDC_H_
#define LIVOX_ROS_DRIVER_LDDC_H_

#include <string>
#include <memory>
#include "lds.h"
#include "livox_sdk.h"

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "livox_interfaces/msg/custom_point.hpp"
#include "livox_interfaces/msg/custom_msg.hpp"

namespace livox_ros {

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

/** Send pointcloud message Data to ros subscriber or save them in rosbag file */
typedef enum {
  kOutputToRos = 0,
  kOutputToRosBagFile = 1,
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} MessageTypeOfTransfer;

class Lddc {
public:
  Lddc(rclcpp::Node * node, int format, int multi_topic, int data_src, int output_type, double frq,
       std::string &frame_id);
  ~Lddc();

  int RegisterLds(Lds *lds);
  void DistributeLidarData(void);
  void CreateBagFile(const std::string &file_name);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

  void initializeDiagnostics();

  Lds *lds_;

private:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  int32_t GetPublishStartTime(LidarDevice *lidar, LidarDataQueue *queue,
                              uint64_t *start_time,
                              StoragePacket *storage_packet);
  uint32_t PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle);
  uint32_t PublishPointcloudData(LidarDataQueue *queue, uint32_t packet_num,
                                 uint8_t handle);
  uint32_t PublishCustomPointcloud(LidarDataQueue *queue, uint32_t packet_num,
                                   uint8_t handle);
  uint32_t PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                          uint8_t handle);

  std::shared_ptr<rclcpp::PublisherBase> CreatePublisher(uint8_t msg_type,
    std::string &topic_name);
  std::shared_ptr<rclcpp::PublisherBase> GetCurrentPublisher(uint8_t handle);
  std::shared_ptr<rclcpp::PublisherBase> GetCurrentImuPublisher(uint8_t handle);
  void PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t handle, LidarDevice *lidar);
  void InitPointcloud2MsgHeader(sensor_msgs::msg::PointCloud2& cloud, const std::string& frame_id);
  void FillPointsToPclMsg(PointCloud& pcl_msg, \
      LivoxPointXyzrtl* src_point, uint32_t num);
  void FillPointsToCustomMsg(livox_interfaces::msg::CustomMsg& livox_msg, \
      LivoxPointXyzrtl* src_point, uint32_t num, uint32_t offset_time, \
      uint32_t point_interval, uint32_t echo_num);

  void onDiagnosticsTimer();
  void registerDiagnosticsUpdater(const std::string & broadcast_code);
  void checkTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkVoltage(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkMotor(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkDirty(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkFirmware(diagnostic_updater::DiagnosticStatusWrapper &stat, const std::string &broadcast_code);
  void checkPPSSignal(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkServiceLife(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkFan(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkPTPSignal(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkTimeSync(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void checkConnection(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & broadcast_code);
  void getLidarByBroadcastcode(std::pair<std::string, DeviceInfo*> & lidar, const std::string & broadcast_code);

  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  uint8_t output_type_;
  double publish_frq_;
  uint32_t publish_period_ns_;
  std::string frame_id_;
  bool check_pps_signal_;

  std::shared_ptr<rclcpp::PublisherBase>private_pub_[kMaxSourceLidar];
  std::shared_ptr<rclcpp::PublisherBase>global_pub_;
  std::shared_ptr<rclcpp::PublisherBase>private_imu_pub_[kMaxSourceLidar];
  std::shared_ptr<rclcpp::PublisherBase>global_imu_pub_;
  rclcpp::Node* cur_node_;
  // rclcpp::rosbag::Bag *bag_;

  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
  uint8_t lidar_count_;
  std::set<std::string> registered_code_set_;

  const std::map<int, const char *> temperature_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "High or Low"}, {DiagStatus::ERROR, "Extremely High or Extremely Low"}};

  const std::map<int, const char *> voltage_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "High"}, {DiagStatus::ERROR, "Extremely High"}};

  const std::map<int, const char *> motor_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "Warning State"}, {DiagStatus::ERROR, "Error State, Unable to Work"}};

  const std::map<int, const char *> dirty_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "Dirty or Blocked"}, {DiagStatus::ERROR, "unused"}};

  const std::map<int, const char *> firmware_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "unused"}, {DiagStatus::ERROR, "Firmware is Abnormal, Need to be Upgraded"}};

  const std::map<int, const char *> pps_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "No PPS Signal"}, {DiagStatus::ERROR, "unused"}};

  const std::map<int, const char *> life_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "Warning for Approaching the End of Service Life"}, {DiagStatus::ERROR, "unused"}};

  const std::map<int, const char *> fan_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "Warning State"}, {DiagStatus::ERROR, "unused"}};

  const std::map<int, const char *> ptp_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "No 1588 Signal"}, {DiagStatus::ERROR, "unused"}};

  const std::map<int, const char *> time_sync_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "System time synchronization is abnormal"}, {DiagStatus::ERROR, "unused"}};

};

}  // namespace livox_ros
#endif
