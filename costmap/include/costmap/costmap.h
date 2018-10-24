//
// Created by shivesh on 6/30/18.
//

#ifndef COSTMAP_COSTMAP_H_
#define COSTMAP_COSTMAP_H_

#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "costmap/map_cell.h"
#include "costmap/layer.h"

namespace costmap {
class Costmap : public rclcpp::Node {
 public:
  Costmap(std::string global_frame, std::string base_frame, unsigned int size_x, unsigned int size_y,
          double resolution, unsigned char default_char);

  virtual ~Costmap();

  void update(const geometry_msgs::msg::Pose &, bool rolling_window);

  void loadPlugin(std::shared_ptr<Layer> plugin);

  std::string global_frame_, base_frame_;
  double minx_, miny_, maxx_, maxy_;
  int origin_x_, origin_y_;

  geometry_msgs::msg::Pose map_origin_;

  unsigned int size_x_, size_y_;
  double resolution_;
  bool rolling_window_, origin_init_;

  MapCell *map_cell;

 private:
  std::vector<std::shared_ptr<Layer>> plugins_;
};
}

#endif //COSTMAP_COSTMAP_H_
