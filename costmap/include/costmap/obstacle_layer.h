//
// Created by shivesh on 7/2/18.
//

#ifndef COSTMAP_OBSTACLE_LAYER_H_
#define COSTMAP_OBSTACLE_LAYER_H_

#include "rclcpp/rclcpp.hpp"
#include "costmap/layer.h"

namespace costmap
{
  class ObstacleLayer : public Layer, rclcpp::Node
  {
   public:
    ObstacleLayer();
    ~ObstacleLayer();
  };
}

#endif //COSTMAP_OBSTACLE_LAYER_H_
