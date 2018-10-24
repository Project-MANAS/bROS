//
// Created by shivesh on 7/1/18.
//

#ifndef COSTMAP_MAP_LAYER_H_
#define COSTMAP_MAP_LAYER_H_

#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"

#include "costmap/map_cell.h"
#include "costmap/layer.h"

namespace costmap {
class MapLayer : public costmap::Layer, rclcpp::Node {
 public:
  MapLayer();

  virtual ~MapLayer();

  virtual void initialise(std::string global_frame,
                          unsigned int size_x,
                          unsigned int size_y,
                          unsigned int origin_x,
                          unsigned int origin_y,
                          double resolution,
                          bool rolling_window);

  virtual void updateBounds(double *min_x, double *max_x, double *min_y, double *max_y);

  virtual void
  updateCosts(MapCell *mc, double *min_x, double *max_x, double *min_y, double *max_y);

 private:
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void callback();

  std::string topic_, map_frame_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  nav_msgs::msg::Odometry pose_;
};
}
#endif //COSTMAP_MAP_LAYER_H_
