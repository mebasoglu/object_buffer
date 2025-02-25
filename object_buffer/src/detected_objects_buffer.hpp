#pragma once
#include "object_buffer.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

namespace object_buffer
{

class ObjectBuffer : public BaseBuffer<DetectedObjects>
{
public:
  explicit ObjectBuffer(const rclcpp::NodeOptions & node_options);

private:
  void onObjects(const DetectedObjects & msg) const;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  struct NodeParam
  {
    bool is_fixed_label{};
    std::string fixed_label{};
    bool is_fixed_size{};
    double size_x{};
    double size_y{};
    double size_z{};
  };
  NodeParam node_param_{};
};

}  // namespace object_buffer