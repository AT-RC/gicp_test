#include "small_gicp_relocalization/small_gicp_relocalization.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 回归单线程执行器，确保系统调度简单稳定
  rclcpp::NodeOptions options;
  auto node = std::make_shared<small_gicp_relocalization::SmallGicpRelocalizationNode>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
