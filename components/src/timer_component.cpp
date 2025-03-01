//  Copyright 2025 Walter Lucetti
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
////////////////////////////////////////////////////////////////////////////////

#include "timer_component.hpp"
#include <chrono>

namespace tc
{

TimerComponent::TimerComponent(const rclcpp::NodeOptions & options)
: Node("timer_component", options), count_(0)
{
  RCLCPP_INFO(get_logger(), "*****************************************");
  RCLCPP_INFO(get_logger(), " ROS 2 Composition Test: Timer Component ");
  RCLCPP_INFO(get_logger(), "*****************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "*****************************************");

  // Create a timer that calls the callback every second
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&TimerComponent::on_timer, this));
}

TimerComponent::~TimerComponent()
{
  RCLCPP_INFO(this->get_logger(), "Destroying TimerComponent...");

  // Kill timer
  timer_.reset();

  RCLCPP_INFO(this->get_logger(), "TimerComponent destroyed.");
}

void TimerComponent::on_timer()
{
  RCLCPP_INFO(this->get_logger(), "TimerComponent: %lu", count_++);
}
}  // namespace tc

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tc::TimerComponent)
