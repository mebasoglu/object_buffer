#include "detected_objects_with_feature_buffer.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"

namespace object_buffer
{
using autoware::universe_utils::updateParam;

DetectedObjectsWithFeatureBuffer::DetectedObjectsWithFeatureBuffer(
  const rclcpp::NodeOptions & node_options)
: BaseBuffer<DetectedObjectsWithFeature>("detected_objects_with_feature_buffer", node_options)
{
  RCLCPP_INFO_STREAM(get_logger(), "Hello");

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DetectedObjectsWithFeatureBuffer::onSetParam, this, std::placeholders::_1));

  node_param_.is_fixed_label = declare_parameter<bool>("is_fixed_label");
  node_param_.fixed_label = declare_parameter<std::string>("fixed_label");

  sub_objects_ = create_subscription<DetectedObjectsWithFeature>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&DetectedObjectsWithFeatureBuffer::onObjects, this, std::placeholders::_1));

  pub_objects_ = create_publisher<DetectedObjectsWithFeature>("~/output/objects", 1);
}

void DetectedObjectsWithFeatureBuffer::onObjects(const DetectedObjectsWithFeature & msg) const
{
  DetectedObjectsWithFeature output_objects;
  output_objects.header = msg.header;

  for (auto object : msg.feature_objects) {
    if (node_param_.is_fixed_label) {
      object.object.classification.at(0).label =
        autoware::object_recognition_utils::toLabel(node_param_.fixed_label);
    }
    output_objects.feature_objects.push_back(object);
  }

  pub_objects_->publish(output_objects);
}

rcl_interfaces::msg::SetParametersResult DetectedObjectsWithFeatureBuffer::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      updateParam(params, "is_fixed_label", p.is_fixed_label);
      updateParam(params, "fixed_label", p.fixed_label);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace object_buffer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(object_buffer::DetectedObjectsWithFeatureBuffer)
