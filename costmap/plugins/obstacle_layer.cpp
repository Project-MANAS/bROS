//
// Created by squadrick on 10th October 2018
//

#include "costmap/obstacle_layer.h"

#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace costmap {
ObstacleLayer::ObstacleLayer() : Node("obstacle_layer"), ros_clock_(RCL_ROS_TIME) {
  RCLCPP_INFO(this->get_logger(), "Using costmap's ObstacleLayer");
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      [this](const nav_msgs::msg::Odometry::SharedPtr pose) {
        pose_ = *pose;
      });
}

ObstacleLayer::~ObstacleLayer() {
  while (clearing_buffers_.size() > 0)
    clearing_buffers_.pop_back();

  while (marking_buffers_.size() > 0)
    marking_buffers_.pop_back();

  while (observation_subscriptions_.size() > 0)
    observation_subscriptions_.pop_back();

  while (observation_buffers_.size() > 0)
    observation_buffers_.pop_back();

  delete[] map_;
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
  map_ = new MapCell[size_x_ * size_y_];

//   TODO (Squadrick): Add brosdb support
  std::vector<std::string> topic_names;

  for (auto it = topic_names.begin(); it != topic_names.end(); ++it) {
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
                                  *tf_buffer_,
                                  global_frame_,
                                  sensor_frame,
                                  &ros_clock_)));

    if (clearing) {
      clearing_buffers_.push_back(observation_buffers_.back());
    }

    if (marking) {
      marking_buffers_.push_back(observation_buffers_.back());
    }

    observation_subscriptions_.push_back(
        this->create_subscription<sensor_msgs::msg::PointCloud2>(*it,
                                                                 std::bind(&ObstacleLayer::incomingPointCloud,
                                                                           this,
                                                                           std::placeholders::_1)));
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

void ObstacleLayer::incomingPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud) {
  // (Squadrick): Locking the Buffer, since the costmap update loop runs asynchronously
  auto buffer = *(observation_buffers_.end());
  buffer->lock();
  buffer->bufferCloud(*pointcloud);
  buffer->unlock();
}

void ObstacleLayer::updateBounds(double *min_x, double *max_x, double *min_y, double *max_y) {
  double robot_x = pose_.pose.pose.position.x;
  double robot_y = pose_.pose.pose.position.y;

  // update origin
  origin_x_ = robot_x - gridsToMetres(size_x_) / 2;
  origin_y_ = robot_y - gridsToMetres(size_y_) / 2;
}

void ObstacleLayer::updateCosts(MapCell *mc, double *minx, double *maxx, double *miny,
                                double *maxy) {
  std::vector<Observation> clearing_observations, marking_observations;

  for (auto buffer = clearing_buffers_.begin(); buffer != clearing_buffers_.end(); ++buffer) {
    (*buffer)->lock();
    (*buffer)->getObservations(clearing_observations);
    (*buffer)->unlock();
  }

  for (auto buffer = marking_buffers_.begin(); buffer != marking_buffers_.end(); ++buffer) {
    (*buffer)->lock();
    (*buffer)->getObservations(marking_observations);
    (*buffer)->unlock();
  }

  for (auto it = clearing_observations.begin(); it != clearing_observations.end(); ++it) {
    // NOTE: Pls don't convert &(*it), it won't work
    raytraceFreespace(&(*it), minx, maxx, miny, maxy);
  }

  for (auto it = marking_observations.begin(); it != marking_observations.end(); ++it) {
    auto square = std::bind((double (*)(double, int)) std::pow, std::placeholders::_1, 2);
    sensor_msgs::msg::PointCloud2 cloud = *(it->cloud_);
    sensor_msgs::PointCloud2Iterator<float> iter_xyz(cloud, "xyz");

    for (; iter_xyz != iter_xyz.end(); ++iter_xyz) {
      double px = iter_xyz[0];
      double py = iter_xyz[1];
      double pz = iter_xyz[2];

      double sq_ob_dist = square(px - it->origin_.x) + square(py - it->origin_.y) + square(pz - it->origin_.z);

      if (sq_ob_dist > square(it->obstacle_range_)) {
        // point is too far away
        continue;
      }
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        // tranform to map failed
        continue;
      }
      mc[mx * size_x_ + my].cost = 100; // LETHAL
      touch(mx, my, minx, miny, maxx, maxy);
    }
  }
}
}

PLUGINLIB_EXPORT_CLASS(costmap::ObstacleLayer, costmap::Layer
)
