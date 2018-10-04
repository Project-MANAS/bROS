//
// Created by shivesh on 7/2/18.
//

#include "costmap/obstacle_layer.h"

#include <pluginlib/class_list_macros.hpp>

namespace costmap {
ObstacleLayer::ObstacleLayer() : Node("obstacle_layer") {
  RCLCPP_INFO(this->get_logger(), "FINALLY");
}

ObstacleLayer::~ObstacleLayer() {

}

void
ObstacleLayer::initialise(unsigned int size_x, unsigned int size_y, unsigned int origin_x, unsigned int origin_y,
                          double resolution, bool rolling_window) {
}

void ObstacleLayer::updateBounds(unsigned int *minx, unsigned int *maxx, unsigned int *miny, unsigned int *maxy,
                                 bool rolling_window) {
}

void ObstacleLayer::updateCosts(MapCell *mc, unsigned int minx, unsigned int maxx, unsigned int miny,
                                unsigned int maxy) {
}
}

PLUGINLIB_EXPORT_CLASS(costmap::ObstacleLayer, costmap::Layer
)
