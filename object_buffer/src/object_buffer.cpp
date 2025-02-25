#include "object_buffer.hpp"

namespace object_buffer
{
ObjectBuffer::ObjectBuffer(const rclcpp::NodeOptions & node_options)
: Node("object_buffer", node_options)
{
}
}  // namespace object_buffer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(object_buffer::ObjectBuffer)
