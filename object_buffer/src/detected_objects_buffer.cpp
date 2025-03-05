#include "detected_objects_buffer.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

namespace object_buffer
{
using autoware::universe_utils::updateParam;

DetectedObjectsBuffer::DetectedObjectsBuffer(const rclcpp::NodeOptions & node_options)
: BaseBuffer<DetectedObjects>("detected_objects_buffer", node_options)
{
  RCLCPP_INFO_STREAM(get_logger(), "Hello");

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DetectedObjectsBuffer::onSetParam, this, std::placeholders::_1));

  node_param_.is_fixed_label = declare_parameter<bool>("is_fixed_label");
  node_param_.fixed_label = declare_parameter<std::string>("fixed_label");
  node_param_.is_fixed_size = declare_parameter<bool>("is_fixed_size");
  node_param_.is_min_size = declare_parameter<bool>("is_min_size");
  node_param_.size_x = declare_parameter<double>("size_x");
  node_param_.size_y = declare_parameter<double>("size_y");
  node_param_.size_z = declare_parameter<double>("size_z");
  node_param_.min_size = declare_parameter<double>("min_size");

  sub_objects_ = create_subscription<DetectedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&DetectedObjectsBuffer::onObjects, this, std::placeholders::_1));

  pub_objects_ = create_publisher<DetectedObjects>("~/output/objects", 1);
}

void DetectedObjectsBuffer::onObjects(const DetectedObjects & msg) const
{
  DetectedObjects output_objects;
  output_objects.header = msg.header;

  for (auto object : msg.objects) {
    if (node_param_.is_fixed_label) {
      object.classification.at(0).label =
        autoware::object_recognition_utils::toLabel(node_param_.fixed_label);
    }
    if (node_param_.is_fixed_size) {
      object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      object.shape.dimensions.x = node_param_.size_x;
      object.shape.dimensions.y = node_param_.size_y;
      object.shape.dimensions.z = node_param_.size_z;
    } else if (node_param_.is_min_size) {
      const auto poly = autoware_utils::to_polygon2d(object);
      const auto area = boost::geometry::area(poly);
      if (area < node_param_.min_size) {
        object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
        object.shape.dimensions.x = node_param_.size_x;
        object.shape.dimensions.y = node_param_.size_y;
        object.shape.dimensions.z = node_param_.size_z;
      }
    }
    output_objects.objects.push_back(object);
  }

  pub_objects_->publish(output_objects);
}

rcl_interfaces::msg::SetParametersResult DetectedObjectsBuffer::onSetParam(
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
      updateParam(params, "is_fixed_size", p.is_fixed_size);
      updateParam(params, "is_min_size", p.is_min_size);
      updateParam(params, "size_x", p.size_x);
      updateParam(params, "size_y", p.size_y);
      updateParam(params, "size_z", p.size_z);
      updateParam(params, "min_size", p.min_size);
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
RCLCPP_COMPONENTS_REGISTER_NODE(object_buffer::DetectedObjectsBuffer)
