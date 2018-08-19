//
// Created by shivesh on 29/6/18.
//

#ifndef COSTMAP_COSTMAP_ROS_H_
#define COSTMAP_COSTMAP_ROS_H_

#include <math.h>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/exceptions.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

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

 protected:
  Costmap* costmap_;

 private:
  void pluginLoader(std::string type);
  void mapUpdateLoop();
  void mapUpdate();
  void computeFreqLoop();
  void velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  bool getRobotPose(geometry_msgs::msg::PoseStamped& pose);

  std::string global_frame_, base_frame_;
  unsigned int size_x_, size_y_;
  double resolution_;
  bool rolling_window_;

  tf2::Duration duration = tf2::Duration(std::chrono::seconds(1));
  tf2_ros::Buffer buffer_;

  pluginlib::ClassLoader<Layer> plugin_loader_;
  std::string plugins_list_;

  double min_freq_, freq_;
  rclcpp::Clock ros_clock_;
  builtin_interfaces::msg::Time last_publish_;
  double transform_tolerance;

  std::thread map_update_thread_;
  bool map_update_thread_shutdown_;

  std::thread compute_freq_thread_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  nav_msgs::msg::Odometry odom_;
  bool vel_init;
};
}

#endif //COSTMAP_COSTMAP_ROS_H_
