// Copyright (c) 2019 Samsung Research America
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

#ifndef NAVIGO_CORE__BEHAVIOR_HPP_
#define NAVIGO_CORE__BEHAVIOR_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "navigo_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "navigo_costmap_2d/costmap_topic_collision_checker.hpp"

namespace navigo_core
{

/**
 * @class Behavior
 * @brief Abstract interface for behaviors to adhere to with pluginlib
 */
class Behavior
{
public:
  using Ptr = std::shared_ptr<Behavior>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Behavior() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<navigo_costmap_2d::CostmapTopicCollisionChecker> collision_checker) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate() = 0;
};

}  // namespace navigo_core

#endif  // NAVIGO_CORE__BEHAVIOR_HPP_
