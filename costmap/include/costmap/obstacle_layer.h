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
    virtual void initialise(unsigned int size_x, unsigned int size_y, unsigned int origin_x, unsigned int origin_y,
                            double resolution, bool rolling_window);
    virtual void updateBounds(unsigned int* minx, unsigned int* maxx, unsigned int* miny, unsigned int* maxy, bool rolling_window);
    virtual void updateCosts(MapCell* mc, unsigned int minx, unsigned int maxx, unsigned int miny, unsigned int maxy);
  };
}

#endif //COSTMAP_OBSTACLE_LAYER_H_
