#include "pointcloud2_cutter/pointcloud2_cutter_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pointcloud2_cutter::PointCloud2CutterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

