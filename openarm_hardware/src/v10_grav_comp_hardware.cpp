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

#include "openarm_hardware/v10_grav_comp_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_v10HW_GC::OpenArm_v10HW_GC() = default;

bool OpenArm_v10HW_GC::parse_config(const hardware_interface::HardwareInfo & info)
{
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;
  } else {
    std::string v = it->second;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    hand_ = (v == "true");
  }

  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;
  } else {
    std::string v = it->second;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    can_fd_ = (v == "true");
  }

  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) kp_[i - 1] = std::stod(it->second);
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) kd_[i - 1] = std::stod(it->second);
  }

  it = info.hardware_parameters.find("gravity_comp_scale");
  if (it != info.hardware_parameters.end()) {
    gravity_comp_scale_ = std::stod(it->second);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("OpenArm_v10HW_GC"),
    "Config: CAN=%s prefix=%s hand=%s can_fd=%s grav_scale=%.2f",
    can_interface_.c_str(), arm_prefix_.c_str(),
    hand_ ? "true" : "false", can_fd_ ? "true" : "false",
    gravity_comp_scale_);

  return true;
}

void OpenArm_v10HW_GC::generate_joint_names()
{
  joint_names_.clear();
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    joint_names_.push_back("openarm_" + arm_prefix_ + "joint" + std::to_string(i));
  }
  if (hand_) {
    joint_names_.push_back("openarm_" + arm_prefix_ + "finger_joint1");
  }
}

bool OpenArm_v10HW_GC::init_gravity_comp(const hardware_interface::HardwareInfo & info)
{
  if (gravity_comp_scale_ <= 0.0) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_GC"),
                "gravity_comp_scale=0 — gravity compensation disabled");
    return true;
  }

  if (info.original_xml.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_GC"),
                "original_xml is empty — gravity compensation disabled");
    return true;
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(info.original_xml, kdl_tree)) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_GC"),
                "Failed to parse URDF into KDL tree — gravity compensation disabled");
    return true;
  }

  // Chain: arm base link -> H8 eef base link (fixed joint carries H8 mass automatically)
  // arm_prefix_ is "left_" or "right_" or ""
  const std::string base_link = "openarm_" + arm_prefix_ + "link0";
  const std::string tip_link  = arm_prefix_ + "eef_base_link";

  if (!kdl_tree.getChain(base_link, tip_link, kdl_chain_)) {
    RCLCPP_WARN(
      rclcpp::get_logger("OpenArm_v10HW_GC"),
      "KDL chain [%s -> %s] not found — falling back to arm-only chain",
      base_link.c_str(), tip_link.c_str());

    const std::string fallback_tip = "openarm_" + arm_prefix_ + "link7";
    if (!kdl_tree.getChain(base_link, fallback_tip, kdl_chain_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("OpenArm_v10HW_GC"),
        "KDL chain [%s -> %s] also failed — gravity compensation disabled",
        base_link.c_str(), fallback_tip.c_str());
      return true;
    }
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_GC"),
                "Using arm-only chain — H8 mass not included in gravity comp");
  }

  if (kdl_chain_.getNrOfJoints() != ARM_DOF) {
    RCLCPP_ERROR(
      rclcpp::get_logger("OpenArm_v10HW_GC"),
      "KDL chain has %u joints, expected %zu — gravity compensation disabled",
      kdl_chain_.getNrOfJoints(), ARM_DOF);
    return true;
  }

  kdl_dyn_param_ = std::make_unique<KDL::ChainDynParam>(
      kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

  gravity_comp_ready_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("OpenArm_v10HW_GC"),
    "Gravity compensation ready: chain [%s -> %s], scale=%.2f",
    base_link.c_str(), tip_link.c_str(), gravity_comp_scale_);

  return true;
}

hardware_interface::CallbackReturn OpenArm_v10HW_GC::on_init(
    const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (!parse_config(info)) return CallbackReturn::ERROR;

  generate_joint_names();

  size_t expected = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW_GC"),
                 "Generated %zu joint names, expected %zu",
                 joint_names_.size(), expected);
    return CallbackReturn::ERROR;
  }

  openarm_ = std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS, DEFAULT_RECV_CAN_IDS);

  if (hand_) {
    openarm_->init_gripper_motor(
        DEFAULT_GRIPPER_MOTOR_TYPE, DEFAULT_GRIPPER_SEND_CAN_ID, DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  const size_t total = joint_names_.size();
  pos_commands_.resize(total, 0.0);
  vel_commands_.resize(total, 0.0);
  tau_commands_.resize(total, 0.0);
  pos_states_.resize(total, 0.0);
  vel_states_.resize(total, 0.0);
  tau_states_.resize(total, 0.0);
  last_pos_commands_ = pos_commands_;

  init_gravity_comp(info);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_GC"),
              "OpenArm_v10HW_GC initialised (gravity_comp=%s)",
              gravity_comp_ready_ ? "ON" : "OFF");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW_GC::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OpenArm_v10HW_GC::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &tau_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OpenArm_v10HW_GC::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_commands_[i]);
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]);
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &tau_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW_GC::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW_GC::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW_GC::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto & arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto & gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      pos_states_[ARM_DOF] = motor_radians_to_joint(gripper_motors[0].get_position());
      vel_states_[ARM_DOF] = 0.0;
      tau_states_[ARM_DOF] = 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW_GC::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  command_received_ = false;
  for (size_t i = 0; i < pos_commands_.size(); ++i) {
    if (std::abs(pos_commands_[i] - last_pos_commands_[i]) > 1e-6) {
      command_received_ = true;
      break;
    }
  }

  // Always send when gravity comp is active so tau_gravity stays current
  // as the arm moves (gravity torque changes continuously with pose)
  if (!command_received_ && !gravity_comp_ready_) {
    return hardware_interface::return_type::OK;
  }

  last_pos_commands_ = pos_commands_;

  // Compute gravity torques from actual joint positions
  KDL::JntArray grav_torques(ARM_DOF);
  if (gravity_comp_ready_) {
    KDL::JntArray q(ARM_DOF);
    for (size_t i = 0; i < ARM_DOF; ++i) {
      q(i) = pos_states_[i];
    }
    kdl_dyn_param_->JntToGravity(q, grav_torques);

    // Debug: log every 100 cycles (~1s) so you can monitor without flooding
    static int log_counter = 0;
    if (++log_counter >= 100) {
      log_counter = 0;
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_GC"),
        "[%s] grav_torques(Nm): %s=%.2f %s=%.2f %s=%.2f %s=%.2f %s=%.2f %s=%.2f %s=%.2f",
        arm_prefix_.c_str(),
        joint_names_[0].c_str(), grav_torques(0),
        joint_names_[1].c_str(), grav_torques(1),
        joint_names_[2].c_str(), grav_torques(2),
        joint_names_[3].c_str(), grav_torques(3),
        joint_names_[4].c_str(), grav_torques(4),
        joint_names_[5].c_str(), grav_torques(5),
        joint_names_[6].c_str(), grav_torques(6));
    }
  }

  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    double tau = tau_commands_[i];
    if (gravity_comp_ready_) {
      tau += gravity_comp_scale_ * grav_torques(i);
    }
    arm_params.push_back({kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  if (hand_ && joint_names_.size() > ARM_DOF) {
    double motor_cmd = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all({{GRIPPER_KP, GRIPPER_KD, motor_cmd, 0.0, 0.0}});
  }

  openarm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

double OpenArm_v10HW_GC::joint_to_motor_radians(double joint_value)
{
  return (joint_value / GRIPPER_JOINT_0_POSITION) * GRIPPER_MOTOR_1_RADIANS;
}

double OpenArm_v10HW_GC::motor_radians_to_joint(double motor_radians)
{
  return GRIPPER_JOINT_0_POSITION * (motor_radians / GRIPPER_MOTOR_1_RADIANS);
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    openarm_hardware::OpenArm_v10HW_GC,
    hardware_interface::SystemInterface)
