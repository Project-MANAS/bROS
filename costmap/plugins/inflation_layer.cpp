//
// Created by shivesh on 7/2/18.
//

#include "costmap/inflation_layer.h"

#include <pluginlib/class_list_macros.hpp>

namespace costmap
{
  InflationLayer::InflationLayer() : Node("hel")
  {
    RCLCPP_INFO(this->get_logger(), "FINALLY");
  }
}

PLUGINLIB_EXPORT_CLASS(costmap::InflationLayer, costmap::Layer)
