//
// Created by shivesh on 7/2/18.
//

#ifndef COSTMAP_INFLATION_LAYER_H_
#define COSTMAP_INFLATION_LAYER_H_

#include "rclcpp/rclcpp.hpp"
#include "costmap/layer.h"

namespace costmap
{
  class InflationLayer : public Layer, rclcpp::Node
  {
   public:
    InflationLayer();
  };
}

#endif //COSTMAP_INFLATION_LAYER_H_
