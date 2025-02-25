#include "object_buffer.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"

namespace object_buffer
{
ObjectBuffer::ObjectBuffer(const rclcpp::NodeOptions & node_options)
: Node("object_buffer", node_options)
{
  RCLCPP_INFO_STREAM(get_logger(), "Hello");

  sub_objects_ = create_subscription<DetectedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&ObjectBuffer::onObjects, this, std::placeholders::_1));

  pub_objects_ = create_publisher<DetectedObjects>("~/output/objects", 1);
}

void ObjectBuffer::onObjects(const DetectedObjects & msg) const
{
  DetectedObjects output_objects;
  output_objects.header = msg.header;

  static const auto label = autoware::object_recognition_utils::toLabel("CAR");

  for (auto object : msg.objects) {
    object.classification.at(0).label = label;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 1.0;
    object.shape.dimensions.y = 1.0;
    object.shape.dimensions.z = 1.0;
    output_objects.objects.push_back(object);
  }

  pub_objects_->publish(output_objects);
}

}  // namespace object_buffer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(object_buffer::ObjectBuffer)
