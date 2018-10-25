//
// Created by squadrick on 10th October 2018
//

#include "costmap/obstacle_layer.h"

#include <pluginlib/class_list_macros.hpp>
#include <cmath>

#define SIGN(x) (x < 0.0 ? -1.0 : 1.0)

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
  int a = 5;
  brosdb::set("set", a);
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
    raytraceFreespace(&(*it), mc, minx, maxx, miny, maxy);
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

void ObstacleLayer::raytraceFreespace(const Observation *clearing_observation,
                                      MapCell *map,
                                      double *min_x,
                                      double *min_y,
                                      double *max_x,
                                      double *max_y) {
  double ox = clearing_observation->origin_.x;
  double oy = clearing_observation->origin_.y;
  sensor_msgs::msg::PointCloud2 &cloud = *(clearing_observation->cloud_);

  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0)) {
    return;
  }

  // endpoints of the map
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

//  touch(ox, oy, min_x, min_y, max_x, max_y);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the endpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = metresToGrids(clearing_observation->raytrace_range_);
//    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(map, x0, y0, x1, y1, cell_raytrace_range);

    updateRaytraceBounds(ox, oy, wx, wy, clearing_observation->raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double *min_x, double *min_y, double *max_x, double *max_y) {
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::raytraceLine(MapCell *map, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                                 unsigned int max_length) {
  int dx = x1 - x0;
  int dy = y1 - y0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = SIGN(dx);
  int offset_dy = SIGN(dy) * size_x_;

  unsigned int offset = y0 * size_x_ + x0;

  // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
  double dist = hypot(dx, dy);
  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

  // if x is dominant
  if (abs_dx >= abs_dy) {
    int error_y = abs_dx / 2;
    bresenham2D(map, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int) (scale * abs_dx));
    return;
  }

  // otherwise y is dominant
  int error_x = abs_dy / 2;
  bresenham2D(map, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int) (scale * abs_dy));
}

void ObstacleLayer::bresenham2D(MapCell *map, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                                int offset_b, unsigned int offset, unsigned int max_length) {
  unsigned int end = std::min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i) {
    map[offset].cost = 0;
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int) error_b >= abs_da) {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  map[offset].cost = 0;
}

}

PLUGINLIB_EXPORT_CLASS(costmap::ObstacleLayer, costmap::Layer
)
