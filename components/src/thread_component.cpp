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

#include "thread_component.hpp"
#include <chrono>

namespace tc
{

ThreadComponent::ThreadComponent(const rclcpp::NodeOptions & options)
: Node("thread_component", options), count_(0), stopThread_(false)
{
  RCLCPP_INFO(get_logger(), "*****************************************");
  RCLCPP_INFO(get_logger(), " ROS 2 Composition Test: Thread Component ");
  RCLCPP_INFO(get_logger(), "*****************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "*****************************************");

  // Create a thread that calls the callback every second
  thread_ = std::make_shared<std::thread>(std::bind(&ThreadComponent::thread_func, this));
}

ThreadComponent::~ThreadComponent()
{
  RCLCPP_INFO(this->get_logger(), "Destroying ThreadComponent...");

  // Force thread stop
  stopThread_ = true;

  // Kill thread
  if (thread_->joinable()) {
    RCLCPP_INFO(this->get_logger(), "Joining thread...");
    thread_->join();
    RCLCPP_INFO(this->get_logger(), "Thread joined.");
  }

  RCLCPP_INFO(this->get_logger(), "ThreadComponent destroyed.");
}

void ThreadComponent::thread_func()
{
  RCLCPP_INFO(this->get_logger(), "ThreadComponent thread started.");
  while (1) {
    // ----> Interruption check
    if (!rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "Ctrl+C received: stopping thread");
      stopThread_ = true;
    }

    if (stopThread_) {
      RCLCPP_INFO(get_logger(), "Thread stopped");
      break;
    }
    // <---- Interruption check

    RCLCPP_INFO(this->get_logger(), "ThreadComponent: %lu", count_++);

    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  RCLCPP_INFO(this->get_logger(), "ThreadComponent thread finished.");
}
}  // namespace tc

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tc::ThreadComponent)
