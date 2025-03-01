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

#ifndef THREAD_COMPONENT_HPP_
#define THREAD_COMPONENT_HPP_

#include "visibility_control.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <thread>
#include <string>
#include <atomic>

namespace tc
{
class ThreadComponent : public rclcpp::Node
{
public:
  TC_COMPONENTS_EXPORT
  explicit ThreadComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~ThreadComponent();

protected:
  void thread_func();

private:
  uint64_t count_;
  std::atomic<bool> stopThread_;
  std::shared_ptr<std::thread> thread_;
};
}  // namespace tc
#endif  // THREAD_COMPONENT_HPP_
