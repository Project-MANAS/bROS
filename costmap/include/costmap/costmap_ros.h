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
#include <costmap/costmap_publisher.h>
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
  bool getRobotPose(geometry_msgs::msg::PoseStamped& pose);
  void mapPublishLoop();
  void computeFreqLoop();
  void velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::string global_frame_, base_frame_;
  unsigned int size_x_, size_y_;
  double resolution_;
  unsigned char default_cost_;
  bool rolling_window_;

  tf2::Duration duration = tf2::Duration(std::chrono::seconds(1));
  tf2_ros::Buffer buffer_;

  pluginlib::ClassLoader<Layer> plugin_loader_;
  std::string plugins_list_;

  rclcpp::Clock ros_clock_;

  double min_update_freq_, update_freq_;
  std::thread map_update_thread_;
  bool updated_;
  double transform_tolerance_;

  double min_publish_freq_, publish_freq_;
  std::thread map_publish_thread_;
  bool map_publish_thread_shutdown_;
  CostmapPublisher* publisher_;

  std::thread compute_freq_thread_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  nav_msgs::msg::Odometry odom_;
  std::string odom_topic_;
  bool vel_init;

  std::string pub_topic_;
};
}

#endif //COSTMAP_COSTMAP_ROS_H_
