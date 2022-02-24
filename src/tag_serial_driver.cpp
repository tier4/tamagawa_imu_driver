/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * tag_serial_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 * Ver 1.00 2019/4/4
 */

#include <fmt/format.h>
#include <boost/algorithm/string.hpp>

#include <tamagawa_imu_driver/tag_serial_driver.h>

static constexpr uint16_t STATUS_MASK = 0x8000;  //!< @brief Bit mask of error flag

TagSerialDriver::TagSerialDriver() : receive_count_(0)
{
  pnh_.param<std::string>("imu_frame_id", imu_frame_id_, "tamagawa/imu_link");
  pnh_.param<std::string>("port", port_, "/dev/imu");

  pub_imu_ = pnh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  updater_.add("imu_data", this, &TagSerialDriver::checkStatus);
  updater_.setHardwareID("tamagawa");

  imu_msg_.header.frame_id = imu_frame_id_;
  imu_msg_.orientation.x = 0.0;
  imu_msg_.orientation.y = 0.0;
  imu_msg_.orientation.z = 0.0;
  imu_msg_.orientation.w = 1.0;

  timer_ = pnh_.createTimer(ros::Rate(1.0), &TagSerialDriver::onTimer, this);

  // Initialize and open serial port
  initialize();
}

TagSerialDriver::~TagSerialDriver()
{
  // Close serial port
  finalize();
}

void TagSerialDriver::initialize()
{
  // Preparation for a subsequent run() invocation
  io_.reset();
  serial_port_ = std::make_shared<asio::serial_port>(io_);
  read_timer_ = std::make_shared<asio::steady_timer>(io_);

  // Open the serial port using the specified device name
  try {
    serial_port_->open(port_);
  } catch (const asio::system_error & e) {
    throw std::runtime_error(fmt::format("Could not open serial port: {}", e.what()));
  }

  // Set options on the serial port
  serial_port_->set_option(asio::serial_port_base::baud_rate(115200));
  serial_port_->set_option(asio::serial_port_base::character_size(8));
  serial_port_->set_option(
    asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
  serial_port_->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
  serial_port_->set_option(
    asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

  // Set the timer's expiry time relative to now
  read_timer_->expires_from_now(std::chrono::seconds(3));
  // Start an asynchronous wait on the timer
  read_timer_->async_wait(
    boost::bind(&TagSerialDriver::onReadTimer, this, asio::placeholders::error));

  boost::thread thr_io(boost::bind(&asio::io_service::run, &io_));

  // Send BIN data request
  std::string data = "$TSC,BIN,30*02\r\n";

  serial_port_->async_write_some(
    asio::buffer(data), boost::bind(
                          &TagSerialDriver::onWrite, this, asio::placeholders::error,
                          asio::placeholders::bytes_transferred));

  receive();
}

void TagSerialDriver::finalize()
{
  serial_port_->close();
  io_.stop();
}

void TagSerialDriver::receive()
{
  // asynchronously read data
  asio::async_read_until(
    *serial_port_, buffer_, "\n",
    boost::bind(
      &TagSerialDriver::onRead, this, asio::placeholders::error,
      asio::placeholders::bytes_transferred));
}

void TagSerialDriver::checkStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (receive_count_ <= 0) {
    stat.summary(DiagStatus::WARN, "Not Received");
    return;
  }

  // Buit In Test error occurred
  if ((imu_status_ & STATUS_MASK) == STATUS_MASK) {
    stat.summary(DiagStatus::ERROR, "Buit In Test error");
    stat.addf("status", "%02X", imu_status_);
    return;
  }

  if (!is_checksum_ok_) {
    stat.summary(DiagStatus::ERROR, "Checksum Error");
    return;
  }

  stat.summary(DiagStatus::OK, "OK");
}

void TagSerialDriver::onRead(const asio::error_code & error, const std::size_t bytes_transfered)
{
  if (error) {
    if (error != asio::error::operation_aborted) {
      ROS_ERROR("Read error: %s", error.message().c_str());
    }
    return;
  } else {
    std::ostringstream oss;
    oss << &buffer_;
    const std::string data = oss.str();

    if (boost::starts_with(data, "$TSC,BIN,") && data.length() == 58) {
      // Data received, so cancel timer operation
      read_timer_->cancel();

      imu_msg_.header.stamp = ros::Time::now();

      int raw_data = ((((data[15] << 8) & 0xFFFFFF00) | (data[16] & 0x000000FF)));
      imu_msg_.angular_velocity.x =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
      raw_data = ((((data[17] << 8) & 0xFFFFFF00) | (data[18] & 0x000000FF)));
      imu_msg_.angular_velocity.y =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
      raw_data = ((((data[19] << 8) & 0xFFFFFF00) | (data[20] & 0x000000FF)));
      imu_msg_.angular_velocity.z =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
      raw_data = ((((data[21] << 8) & 0xFFFFFF00) | (data[22] & 0x000000FF)));
      imu_msg_.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
      raw_data = ((((data[23] << 8) & 0xFFFFFF00) | (data[24] & 0x000000FF)));
      imu_msg_.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
      raw_data = ((((data[25] << 8) & 0xFFFFFF00) | (data[26] & 0x000000FF)));
      imu_msg_.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]

      imu_status_ = ((data[13] << 8) & 0x0000FF00) | (data[14] & 0x000000FF);

      pub_imu_.publish(imu_msg_);

      // verify checksum
      is_checksum_ok_ = verifyChecksum(data);

      ++receive_count_;
    }
  }

  receive();
}

void TagSerialDriver::onReadTimer(const asio::error_code & error)
{
  if (error) {
    ROS_DEBUG("Timer canceled. Data received successfully.");
  } else {
    ROS_WARN("Read timeout. Retrying to connect.");
    finalize();
    initialize();
  }
}

void TagSerialDriver::onWrite(const asio::error_code & error, const std::size_t bytes_transfered)
{
  if (error) {
    ROS_ERROR("Write error: %s", error.message().c_str());
  } else {
    ROS_DEBUG("%ld bytes transfered.", bytes_transfered);
  }
}

void TagSerialDriver::onTimer(const ros::TimerEvent & event) { updater_.force_update(); }

bool TagSerialDriver::verifyChecksum(const std::string & data)
{
  uint8_t sum = 0;
  uint8_t calc_sum = 0;
  const int len = data.length();

  // Ignore 1st byte '$', data after a boundary(* CH CH CR LF)
  for (int i = 1; i < len - 5; ++i) calc_sum = calc_sum ^ data[i];

  const char sum_str[3] = {data[len - 4], data[len - 3]};

  try {
    sum = std::stoi(sum_str, 0, 16);
  } catch (std::exception & ex) {
    return false;
  }

  return (calc_sum == sum);
}
