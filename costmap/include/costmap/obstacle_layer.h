//
// Created by squadrick on 7/2/18.
//

#ifndef COSTMAP_OBSTACLE_LAYER_H_
#define COSTMAP_OBSTACLE_LAYER_H_

#include "costmap/layer.h"
#include "costmap/map_cell.h"
#include "costmap/observation_buffer.h"
#include <nav_msgs/msg/odometry.hpp>

namespace costmap {
class ObstacleLayer : public Layer, rclcpp::Node {
 public:
  ObstacleLayer();

  ~ObstacleLayer();

  virtual void initialise(std::string global_frame,
                          unsigned int size_x,
                          unsigned int size_y,
                          unsigned int origin_x,
                          unsigned int origin_y,
                          double resolution,
                          bool rolling_window);

  virtual void updateBounds(double *min_x, double *max_x, double *min_y, double *max_y);

  virtual void updateCosts(MapCell *mc, double *min_x, double *max_x, double *min_y, double *max_y);
  void raytraceFreespace(Observation *ob,
                         double *mix_x,
                         double *max_x,
                         double *min_y,
                         double *max_y);

 private:
  void incomingPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message);
  void callback();

  rclcpp::Clock ros_clock_;
  std::string global_frame_;
  tf2_ros::Buffer *tf_buffer_;
  std::vector<boost::shared_ptr<ObservationBuffer>> observation_buffers_;
  std::vector<boost::shared_ptr<ObservationBuffer>> marking_buffers_;
  std::vector<boost::shared_ptr<ObservationBuffer>> clearing_buffers_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> observation_subscriptions_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry pose_;
};

}

#endif //COSTMAP_OBSTACLE_LAYER_H_
