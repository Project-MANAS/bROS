//
// Created by shivesh on 7/1/18.
//

#include "costmap/map_layer.h"

#include <pluginlib/class_list_macros.hpp>

namespace costmap
{
  MapLayer::MapLayer() : Node("hello")
  {
    RCLCPP_INFO(this->get_logger(),"FINALLY");
  }
}

PLUGINLIB_EXPORT_CLASS(costmap::MapLayer, costmap::Layer)
