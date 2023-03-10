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

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

#include "br2_tf2_detector/DisplacementMonitorNode.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

using namespace std::chrono_literals;

DisplacementMonitorNode::DisplacementMonitorNode()
: Node("displacement_monitor"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  timer_ = create_wall_timer(
    1000ms, std::bind(&DisplacementMonitorNode::control_cycle, this));
}

void
DisplacementMonitorNode::control_cycle()
{
  geometry_msgs::msg::TransformStamped odom2robot;

  try {
    odom2robot = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Odom transform not found: %s", ex.what());
    return;
  }

  double x = last_pose.transform.translation.x - odom2robot.transform.translation.x;
  double y = last_pose.transform.translation.y - odom2robot.transform.translation.y;
  double z = last_pose.transform.translation.z - odom2robot.transform.translation.z;

	double dist = sqrt(pow(x, 2) + pow(y, 2));       //calculating Euclidean distance
  double theta = atan2(y, x);

  RCLCPP_INFO(
    get_logger(), "The distance covered in the last second was %lf m", dist);

  last_pose = odom2robot;

}

}  // namespace br2_tf2_detector
