#pragma once
#include "rclcpp/rclcpp.hpp"

namespace object_buffer
{

class ObjectBuffer : public rclcpp::Node
{
public:
  explicit ObjectBuffer(const rclcpp::NodeOptions & node_options);

private:
};

}  // namespace object_buffer
