// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <vector>

// KDL for gravity compensation
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief OpenArm V10 Hardware Interface with Gravity Compensation
 *
 * Extends the base V10 hardware with model-based gravity compensation
 * via KDL. The gravity torque is computed from actual joint positions
 * each control cycle and injected into the MIT tau (torque feedforward)
 * field, so the motor's Kp only handles small tracking errors rather
 * than fighting gravity.
 *
 * Plugin name: openarm_hardware/OpenArm_v10HW_GC
 *
 * Extra URDF hardware parameters:
 *   gravity_comp_scale  float  0.0-1.0, default 0.0 (safe ramp-up)
 */
class OpenArm_v10HW_GC : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10HW_GC();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

 private:
  static constexpr size_t ARM_DOF = 7;

  const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM8009,
      openarm::damiao_motor::MotorType::DM8009,
      openarm::damiao_motor::MotorType::DM4340,
      openarm::damiao_motor::MotorType::DM4340,
      openarm::damiao_motor::MotorType::DM4310,
      openarm::damiao_motor::MotorType::DM4310,
      openarm::damiao_motor::MotorType::DM4310,
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

  const openarm::damiao_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_MOTOR_1_RADIANS  = -1.0472;
  const double GRIPPER_KP = 5.0;
  const double GRIPPER_KD = 0.1;

  std::vector<double> kp_ = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
  std::vector<double> kd_ = {2.75, 2.5,  2.0,  2.0,  0.7,  0.6,  0.5};

  std::string can_interface_;
  std::string arm_prefix_;
  bool        hand_   = true;
  bool        can_fd_ = true;

  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  std::vector<std::string> joint_names_;
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;
  std::vector<double> last_pos_commands_;
  bool command_received_ = false;

  // Gravity compensation (KDL)
  KDL::Chain                          kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> kdl_dyn_param_;
  double gravity_comp_scale_ = 0.0;
  bool   gravity_comp_ready_ = false;
  int    log_cnt_ = 0;  // per-instance, not static — avoids shared counter bug

  bool parse_config(const hardware_interface::HardwareInfo & info);
  void generate_joint_names();
  bool init_gravity_comp(const hardware_interface::HardwareInfo & info);

  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}  // namespace openarm_hardware
