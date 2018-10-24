//
// Created by shivesh on 7/1/18.
//

#include "costmap/map_layer.h"

#include <pluginlib/class_list_macros.hpp>

using std::placeholders::_1;

typedef unsigned int uint;

namespace costmap {
MapLayer::MapLayer() : Node("map_layer"), topic_("/map") {
  subscription_ =
      this->create_subscription<nav_msgs::msg::OccupancyGrid>(topic_, std::bind(&MapLayer::incomingMap, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      [this](const nav_msgs::msg::Odometry::SharedPtr pose) {
        pose_ = *pose;
      });
}

MapLayer::~MapLayer() {
  delete[] map_;
}

void MapLayer::initialise(std::string global_frame,
                          unsigned int size_x,
                          unsigned int size_y,
                          unsigned int origin_x,
                          unsigned int origin_y,
                          double resolution,
                          bool rolling_window) {
  size_x_ = size_x;
  size_y_ = size_y;
  unsigned int size = size_x * size_y;
  map_ = new MapCell[size];
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  resolution_ = resolution;
  rolling_window_ = rolling_window;
  minx_ = 0;
  miny_ = 0;
  maxx_ = 0;
  maxy_ = 0;

  std::thread spin_thread = std::thread(&MapLayer::callback, this);
  spin_thread.detach();
}

void MapLayer::callback() {
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
    rate.sleep();
  }
}

void MapLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  if (!map_received_) {
    map_received_ = true;
  }

  unsigned int width = (unsigned int) (map->info.width / resolution_);
  unsigned int height = (unsigned int) (map->info.height / resolution_);

  if (!rolling_window_) {
    minx_ = origin_x_ + (int) ((map->info.origin.position.x) / resolution_);
    miny_ = origin_y_ + (int) ((map->info.origin.position.y) / resolution_);
  } else {
    minx_ = origin_x_ + (int) ((map->info.origin.position.x - pose_.pose.pose.position.x) / resolution_);
    miny_ = origin_y_ + (int) ((map->info.origin.position.y - pose_.pose.pose.position.y) / resolution_);
  }
  maxx_ = minx_ + width;
  maxy_ = miny_ + height;

  for (unsigned int i = miny_; i < maxy_; ++i) {
    for (unsigned int j = minx_; j < maxx_; ++j) {
      int index = (int) ((i - miny_) * resolution_) * map->info.width + (int) ((j - minx_) * resolution_);
      map_[i * size_x_ + j].cost = (unsigned char) map->data[index];
    }
  }
}

void MapLayer::updateBounds(double *min_x, double *max_x, double *min_y, double *max_y) {
  if (rolling_window_)
    return;

  double wx, wy;

  mapToWorld(minx_, miny_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(minx_ + size_x_, miny_ + size_y_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);
}

void
MapLayer::updateCosts(MapCell *mc, double *minx, double *maxx, double *miny, double *maxy) {
  for (uint i = static_cast<uint>(*miny); i < static_cast<uint>(*maxy); ++i) {
    for (uint j = static_cast<uint>(*minx); j < static_cast<uint>(*maxx); ++j) {
      mc[i * size_x_ + j].cost = std::max(mc[i * size_x_ + j].cost, map_[i * size_x_ + j].cost);
    }
  }
}

}

PLUGINLIB_EXPORT_CLASS(costmap::MapLayer, costmap::Layer
)
