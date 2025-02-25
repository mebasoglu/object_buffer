#pragma once
#include "object_buffer.hpp"

#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"

namespace object_buffer
{
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;

class DetectedObjectsWithFeatureBuffer : public BaseBuffer<DetectedObjectsWithFeature>
{
public:
  explicit DetectedObjectsWithFeatureBuffer(const rclcpp::NodeOptions & node_options);

private:
  void onObjects(const DetectedObjectsWithFeature & msg) const;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  struct NodeParam
  {
    bool is_fixed_label{};
    std::string fixed_label{};
  };
  NodeParam node_param_{};
};

}  // namespace object_buffer
