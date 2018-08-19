//
// Created by shivesh on 7/1/18.
//

#ifndef COSTMAP_MAP_LAYER_H_
#define COSTMAP_MAP_LAYER_H_

#include "rclcpp/rclcpp.hpp"
#include "costmap/layer.h"

namespace costmap
{
  class MapLayer : public costmap::Layer, rclcpp::Node
  {
   public:
    MapLayer();
    ~MapLayer();
  };
}
#endif //COSTMAP_MAP_LAYER_H_
