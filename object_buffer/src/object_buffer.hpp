#pragma once
#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <memory>

namespace object_buffer
{
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;

template <typename T>
class BaseBuffer : public rclcpp::Node
{
public:
  BaseBuffer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
  : Node(node_name, node_options) {};

protected:
  virtual void onObjects(const T & msg) const = 0;
  virtual rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params) = 0;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  typename rclcpp::Subscription<T>::SharedPtr sub_objects_;
  typename rclcpp::Publisher<T>::SharedPtr pub_objects_;
};

}  // namespace object_buffer
