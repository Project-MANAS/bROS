//
// Created by squadrick on 10th October 2018
//

#include "costmap/obstacle_layer.h"

#include <pluginlib/class_list_macros.hpp>

namespace costmap {
ObstacleLayer::ObstacleLayer() : Node("obstacle_layer"), ros_clock_(ROS_RCL_TIME) {
  RCLCPP_INFO(this->get_logger(), "Using costmap's ObstacleLayer");
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      [this](const nav_msgs::msg::Odometry::SharedPtr pose) {
        pose_ = *pose;
      });
}

ObstacleLayer::~ObstacleLayer() {
  while(clearing_observations_.size() > 0)
    clearning_observations_.pop_back();

  while(marking_observations_.size() > 0)
    marking_observations_.pop_back();

  while(observation_subscriptions_.size() > 0)
    observation_subscriptions_.pop_back();

  while(observation_buffers_.size() > 0)
    observation_buffers_.pop_back();

  delete [] map_cell_;
}

void
ObstacleLayer::initialise(std::string global_frame,
                          unsigned int size_x,
                          unsigned int size_y,
                          unsigned int origin_x,
                          unsigned int origin_y,
                          double resolution,
                          bool rolling_window) {

  global_frame_ = global_frame;
  size_x_ = size_x;
  size_y_ = size_y;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  resolution_ = resolution;
  rolling_window_ = rolling_window;
  map_cell_ = new MapCell[size_x_ * size_y_];

//   TODO (Squadrick): Add brosdb support
  std::vector<std::string> topic_names;

  for (auto it = topic_names.begin(); it != topics_names.end(); ++it) {
    double observation_keep_time = 0.0;
    double expected_update_rate = 0.0;
    double min_obstacle_height = 0.0;
    double max_obstacle_height = 2.0;
    double obstacle_range = 2.5;
    double raytrace_range = 3.0;
    bool clearing = true;
    bool marking = true;
    std::string sensor_frame = "";

    observation_buffers_.push_back(
        boost::shared_ptr<ObservationBuffer>(
            new ObservationBuffer(*it,
                                  observation_keep_time,
                                  expected_update_rate,
                                  min_obstacle_height,
                                  max_obstacle_height,
                                  obstacle_range,
                                  raytrace_range,
                                  *tf_,
                                  global_frame_,
                                  sensor_frame,
                                  ros_clock_)));

    if (clearing) {
      clearing_buffers_.push_back(observation_buffers_.back());
    }

    if (marking) {
      marking_buffers_.push_back(observation_buffers_.back());
    }

    observation_subscriptions_.push_back(
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            *it,
            std::bind(
                ObstacleLayer::pointCloud2Callback,
                this,
                _1,
                observation_buffers_.back())));
  }
  std::thread spin_thread = std::thread(&ObstacleLayer::callback, this);
  spin_thread.detach();
}

void ObstacleLayer::callback() {
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
    rate.sleep();
  }
}

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::msg::PointCloud2ConstPtr &message,
                                        const boost::shared_ptr<ObservationBuffer> &buffer) {
  // Squadrick: Locking the Buffer, since the costmap update loop runs asynchronously
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void ObstacleLayer::updateBounds(unsigned int *minx, unsigned int *maxx, unsigned int *miny, unsigned int *maxy,
                                 bool rolling_window) {

}

void ObstacleLayer::updateCosts(MapCell *mc, unsigned int minx, unsigned int maxx, unsigned int miny,
                                unsigned int maxy) {

}

void MapLayer::updatePose(const nav_msgs::msg::Odometry::SharedPtr pose) {
  pose_ = *pose;
}

}

PLUGINLIB_EXPORT_CLASS(costmap::ObstacleLayer, costmap::Layer
)
