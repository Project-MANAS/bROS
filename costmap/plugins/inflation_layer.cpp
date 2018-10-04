//
// Created by shivesh on 7/2/18.
//

#include "costmap/inflation_layer.h"

#include <pluginlib/class_list_macros.hpp>

namespace costmap {
InflationLayer::InflationLayer() : Node("inflation_layer") {
  RCLCPP_INFO(this->get_logger(), "FINALLY");
}

InflationLayer::~InflationLayer() {

}

void
InflationLayer::initialise(unsigned int size_x, unsigned int size_y, unsigned int origin_x, unsigned int origin_y,
                           double resolution, bool rolling_window) {
}

void InflationLayer::updateBounds(unsigned int *minx, unsigned int *maxx, unsigned int *miny, unsigned int *maxy,
                                  bool rolling_window) {
}

void InflationLayer::updateCosts(MapCell *mc, unsigned int minx, unsigned int maxx, unsigned int miny,
                                 unsigned int maxy) {
}
}

PLUGINLIB_EXPORT_CLASS(costmap::InflationLayer, costmap::Layer
)
