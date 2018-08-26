//
// Created by shivesh on 7/1/18.
//

#include "costmap/map_layer.h"

#include <pluginlib/class_list_macros.hpp>

using std::placeholders::_1;

namespace costmap
{
  MapLayer::MapLayer() :
    Node("map_layer"),
    topic_("/map"),
    minx_(0),
    maxx_(0),
    miny_(0),
    maxy_(0),
    map_received_(false)
  {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_, std::bind(&MapLayer::incomingMap, this, _1));
  }

  MapLayer::~MapLayer(){
    delete[] map_cell;
  }

  void MapLayer::initialise(unsigned int size_x, unsigned int size_y, unsigned int origin_x, unsigned int origin_y){
    size_x_ = size_x;
    size_y_ = size_y;
    unsigned int size = size_x * size_y;
    map_cell = new MapCell[size];
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    std::thread spin_thread = std::thread(&MapLayer::callback, this);
    spin_thread.detach();
  }

  void MapLayer::callback() {
    rclcpp::Rate rate(10.0);
    while(rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }
  }

  void MapLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
    if(!map_received_){
      map_received_ = true;
      initial_x_ = map->info.origin.position.x;
      initial_y_ = map->info.origin.position.y;
    }

    double resolution = map->info.resolution;
    unsigned int origin_x = origin_x_ + (int) ((map->info.origin.position.x - initial_x_) / resolution);
    unsigned int origin_y = origin_y_ + (int) ((map->info.origin.position.y - initial_y_) / resolution);
    unsigned int width = map->info.width, height = map->info.height;

    RCLCPP_INFO(this->get_logger(),"actual origin x %f origin y%f", map->info.origin.position.x, map->info.origin.position.y);

    unsigned int index = 0;
    minx_ = origin_x - width / 2 - 1;
    maxx_ = minx_ + width;
    miny_ = origin_y - height / 2 - 1;
    maxy_ = miny_ + height;

    for(unsigned int i = miny_; i < maxy_; ++i)
    {
      for(unsigned int j = minx_; j < maxx_; ++j)
      {
        map_cell[i * size_x_ + j].cost = (unsigned char) map->data[index++];
      }
    }
  }

  void MapLayer::updateBounds(unsigned int* minx, unsigned int* maxx, unsigned int* miny, unsigned int* maxy,
      double* origin_x, double* origin_y, bool rolling_window){
    if(rolling_window){
      return;
    }
    *origin_x = initial_x_;
    *origin_y = initial_y_;
    *minx = std::max(*minx, minx_);
    *maxx = std::min(*maxx, maxx_);
    *miny = std::max(*miny, miny_);
    *maxy = std::min(*maxy, maxy_);
  }

  void MapLayer::updateCosts(MapCell* mc, unsigned int minx, unsigned int maxx, unsigned int miny, unsigned int maxy)
  {
    for(unsigned int i = miny; i < maxy; ++i)
    {
      for(unsigned int j = minx; j < maxx; ++j)
      {
        mc[i * size_x_ + j].cost = std::max(mc[i * size_x_ + j].cost, map_cell[i * size_x_ + j].cost);
      }
    }
  }

}

PLUGINLIB_EXPORT_CLASS(costmap::MapLayer, costmap::Layer)
