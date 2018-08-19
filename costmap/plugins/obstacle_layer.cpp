//
// Created by shivesh on 7/2/18.
//

#include "costmap/obstacle_layer.h"

#include <pluginlib/class_list_macros.hpp>

namespace costmap
{
  ObstacleLayer::ObstacleLayer() : Node("obstacle_layer")
  {
    RCLCPP_INFO(this->get_logger(), "FINALLY");
  }

  ObstacleLayer::~ObstacleLayer(){

  }
}

PLUGINLIB_EXPORT_CLASS(costmap::ObstacleLayer, costmap::Layer)
