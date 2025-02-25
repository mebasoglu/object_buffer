#pragma once
#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <memory>

namespace object_buffer
{
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;

class ObjectBuffer : public rclcpp::Node
{
public:
  explicit ObjectBuffer(const rclcpp::NodeOptions & node_options);

private:
  void onObjects(const DetectedObjects & msg) const;

  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_;
};

}  // namespace object_buffer
