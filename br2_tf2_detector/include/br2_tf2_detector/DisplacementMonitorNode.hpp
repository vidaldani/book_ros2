// Copyright 2021 Intelligent Robotics Lab
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

#ifndef BR2_TF2_DETECTOR__DisplacementMonitorNode_HPP_
#define BR2_TF2_DETECTOR__DisplacementMonitorNode_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

class DisplacementMonitorNode : public rclcpp::Node
{
public:
  DisplacementMonitorNode();

private:
  void control_cycle();
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::TransformStamped last_pose;
};

}  // namespace br2_tf2_detector
#endif  // BR2_TF2_DETECTOR__DisplacementMonitorNode_HPP_
