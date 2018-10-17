//
// Created by shivesh on 7/2/18.
//

#ifndef COSTMAP_OBSTACLE_LAYER_H_
#define COSTMAP_OBSTACLE_LAYER_H_

#include "rclcpp/rclcpp.hpp"
#include "costmap/layer.h"
#include "costmap/map_cell.h"
#include "costmap/observation_buffer.h"

namespace costmap {
class ObstacleLayer : public Layer, rclcpp::Node {
 public:
  ObstacleLayer();

  ~ObstacleLayer();

  virtual void initialise(unsigned int size_x, unsigned int size_y, unsigned int origin_x, unsigned int origin_y,
                          double resolution, bool rolling_window);

  virtual void updateBounds(unsigned int *minx, unsigned int *maxx, unsigned int *miny, unsigned int *maxy,
                            bool rolling_window);

  virtual void updateCosts(MapCell *mc, unsigned int minx, unsigned int maxx, unsigned int miny, unsigned int maxy);

 private:

  void callback();

  unsigned int size_x, size_y_, origin_x_, origin_y_;
  double resolution_;
  bool rolling_window_;

  MapCell *map_cell_;

  rclcpp::Clock ros_clock_;
  std::string global_frame_;
  tf2_buffer *tf_buffer_;
  nav_msgs::msg::Odometry pose_;
  std::vector<boost::shared_ptr<ObservationBuffer>> observation_buffers_;
  std::vector<boost::shared_ptr<ObservationBuffer>> marking_buffers_;
  std::vector<boost::shared_ptr<ObservationBuffer>> clearing_buffers_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> observation_subscriptions_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}

#endif //COSTMAP_OBSTACLE_LAYER_H_
