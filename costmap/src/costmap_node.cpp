//
// Created by shivesh on 29/6/18.
//

#include "costmap/costmap_ros.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto costmap_node = std::make_shared<costmap::CostmapROS>();
  rclcpp::spin(costmap_node);
  rclcpp::shutdown();
  return 0;
}
