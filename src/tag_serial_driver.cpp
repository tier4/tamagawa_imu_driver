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

#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rate_bound_status.hpp"

#include <sys/ioctl.h>
#include <memory>
#include <thread>


std::shared_ptr<custom_diagnostic_tasks::RateBoundStatus> rate_bound_status;
std::unique_ptr<diagnostic_updater::Updater> diag_updater;


#include <boost/asio.hpp>
using namespace boost::asio;

void loop_process(
  rclcpp::Node::SharedPtr node,
  boost::asio::serial_port &serial_port,
  std::string imu_frame_id,
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub,
  std::shared_ptr<custom_diagnostic_tasks::RateBoundStatus> rate_bound_status
)
{
  std::size_t length;
  int raw_data;
  std::string rbuf;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  while (rclcpp::ok()) {
    boost::asio::streambuf response;
    try {
      boost::asio::read_until(serial_port, response, "\n");
      rbuf = std::string(
        boost::asio::buffers_begin(response.data()), boost::asio::buffers_end(response.data()));
      length = rbuf.size();
    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(rclcpp::get_logger("tag_serial_driver"), "Error reading from serial port: %s", e.what());
      continue;
    }


    if (length > 0) {
      if (rbuf[5] == 'B' && rbuf[6] == 'I' && rbuf[7] == 'N' && rbuf[8] == ',' && length == 58) {
        imu_msg.header.frame_id = imu_frame_id;
        imu_msg.header.stamp = node->now();

        raw_data = ((((rbuf[15] << 8) & 0xFFFFFF00) | (rbuf[16] & 0x000000FF)));
        imu_msg.angular_velocity.x =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[17] << 8) & 0xFFFFFF00) | (rbuf[18] & 0x000000FF)));
        imu_msg.angular_velocity.y =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[19] << 8) & 0xFFFFFF00) | (rbuf[20] & 0x000000FF)));
        imu_msg.angular_velocity.z =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[21] << 8) & 0xFFFFFF00) | (rbuf[22] & 0x000000FF)));
        imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[23] << 8) & 0xFFFFFF00) | (rbuf[24] & 0x000000FF)));
        imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[25] << 8) & 0xFFFFFF00) | (rbuf[26] & 0x000000FF)));
        imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]

        pub->publish(imu_msg);
        rate_bound_status->tick();

      } else if (rbuf[5] == 'V' && rbuf[6] == 'E' && rbuf[7] == 'R' && rbuf[8] == ',') {
        RCLCPP_DEBUG(rclcpp::get_logger("tag_serial_driver"), "%s", rbuf.c_str());
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tag_serial_driver");
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1000);

  std::string imu_frame_id = node->declare_parameter<std::string>("imu_frame_id", "imu");
  std::string port = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");

  io_service io;
  serial_port serial_port(io);
  try {
    serial_port.open(port);
    serial_port.set_option(serial_port_base::baud_rate(115200));
    serial_port.set_option(serial_port_base::character_size(8));
    serial_port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    serial_port.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  } catch (boost::system::system_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger("tag_serial_driver"), "Error opening serial port: %s", e.what());
    return 1;
  }

  std::string wbuf = "$TSC,BIN,30\x0d\x0a";
  serial_port.write_some(buffer(wbuf));

  rclcpp::Rate loop_rate(30.0);

  auto frequency_reference = node->declare_parameter<double>("frequency_reference", 200.0);
  auto ok_min_freq = node->declare_parameter<double>(
    "diagnostics.rate_bound_status.frequency_ok.min_hz", 100.0);
  auto ok_max_freq = node->declare_parameter<double>(
    "diagnostics.rate_bound_status.frequency_ok.max_hz", 10000.0);
  auto warn_min_freq = node->declare_parameter<double>(
    "diagnostics.rate_bound_status.frequency_warn.min_hz", 50.0);
  auto warn_max_freq = node->declare_parameter<double>(
    "diagnostics.rate_bound_status.frequency_warn.max_hz", 100000.0);
  node->declare_parameter<bool>("diagnostic_updater.use_fqn", true);  // read by diagnostic updater
  rate_bound_status = std::make_shared<custom_diagnostic_tasks::RateBoundStatus>(
    node.get(), custom_diagnostic_tasks::RateBoundStatusParam(ok_min_freq, ok_max_freq),
    custom_diagnostic_tasks::RateBoundStatusParam(warn_min_freq, warn_max_freq), 2, false);
  diag_updater = std::make_unique<diagnostic_updater::Updater>(node);
  diag_updater->setHardwareID(imu_frame_id);
  diag_updater->setPeriod(1.0 / frequency_reference);
  diag_updater->add(*rate_bound_status);

  std::thread loop_thread(loop_process, node, std::ref(serial_port), imu_frame_id, pub, rate_bound_status);

  rclcpp::spin(node);

  if (loop_thread.joinable()) {
    loop_thread.join();
  }

  return 0;
}
