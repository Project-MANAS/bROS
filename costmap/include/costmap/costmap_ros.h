//
// Created by shivesh on 29/6/18.
//

#ifndef COSTMAP_COSTMAP_ROS_H_
#define COSTMAP_COSTMAP_ROS_H_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/exceptions.hpp"

#include <costmap/layer.h>
#include <costmap/costmap.h>
#include <costmap/map_layer.h>

namespace costmap
{
class CostmapROS : public rclcpp::Node
{
 public:
  CostmapROS();
  virtual ~CostmapROS();

  Costmap* getCostmap()
  {
    return costmap_;
  }

 protected:
  Costmap* costmap_;

 private:
  void pluginLoader(std::string type);

  std::string global_frame_, base_frame_;
  double size_x_, size_y_, resolution_;
  //tf2_ros::Buffer buffer_;
  pluginlib::ClassLoader<Layer> plugin_loader_;
  std::string plugins_list_;
};
}

#endif //COSTMAP_COSTMAP_ROS_H_
