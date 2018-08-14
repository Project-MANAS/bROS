//
// Created by shivesh on 29/6/18.
//

#include "costmap/costmap_ros.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto costmap_node = std::make_shared<costmap::CostmapROS>();
  exec.add_node(costmap_node);
  exec.spin();
  rclcpp::shutdown();
  return (0);
}
