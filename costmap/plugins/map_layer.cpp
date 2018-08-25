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
    map_received_(false)
  {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_, std::bind(&MapLayer::incomingMap, this, _1));
  }

  MapLayer::~MapLayer(){
    delete[] map_cell;
  }

  void MapLayer::initialise(unsigned int size_x, unsigned int size_y, unsigned int origin_x, unsigned int origin_y){
    rclcpp::Rate rate(1.0);
    unsigned int size = size_x * size_y;
    map_cell = new MapCell[size];
    minx_ = miny_ =  maxx_ = maxy_ = 0;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    while(rclcpp::ok() && !map_received_) {
      executor.spin_some();
      rate.sleep();
    }
  }

  void MapLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
    map_received_ = true;
    double resolution = map->info.resolution;
    unsigned int origin_x = origin_x_ - map->info.origin.position.x / resolution;
    unsigned int origin_y = origin_y_ - map->info.origin.position.y / resolution;
    unsigned int width = map->info.width, height = map->info.height;

    RCLCPP_INFO(this->get_logger(),"originx%d", map->info.origin.position.x);
    RCLCPP_INFO(this->get_logger(),"originy%d", map->info.origin.position.y);
    RCLCPP_INFO(this->get_logger(),"width%d", map->info.width);
    RCLCPP_INFO(this->get_logger(),"height%d", map->info.height);
    unsigned int index = 0;
    for(unsigned int i = origin_y - height / 2; i <= origin_y + height / 2; ++i){
      for(unsigned int j = origin_x - width / 2; j <= origin_x + width / 2; ++j){
        //RCLCPP_INFO(this->get_logger(),"%d",i*width +j);
        map_cell[i * width + j].cost = (unsigned char) map->data[index++];
      }
    }

    minx_ = origin_x - width / 2;
    maxx_ = origin_x + width / 2;
    miny_ = origin_y - height / 2;
    maxy_ = origin_y + height / 2;
    RCLCPP_INFO(this->get_logger(),"%d, %d, %d, %d", minx_, maxx_, miny_, maxy_);
    map_frame_ = map->header.frame_id;
    width_ = map->info.width;
    height_ = map->info.height;
  }

  void MapLayer::updateBounds(unsigned int origin_x, unsigned int origin_y, double yaw, unsigned int* minx,
                              unsigned int* maxx, unsigned int* miny, unsigned int* maxy, bool rolling_window){
    if(rolling_window){
      return;
    }
    *minx = std::max(*minx, minx_);
    *maxx = std::min(*maxx, maxx_);
    *miny = std::max(*miny, miny_);
    *maxy = std::min(*maxy, maxy_);
  }

  void MapLayer::updateCosts(MapCell* mc, unsigned int minx, unsigned int maxx, unsigned int miny, unsigned int maxy,
                             unsigned int size_x, unsigned int size_y){
    for(unsigned int i = miny; i <= maxy; ++i){
      for(unsigned int j = minx; j <= maxx; ++j){
        mc[i*size_y + j].cost = std::max(mc[i*size_y + j].cost, map_cell[i*size_y + j].cost);
      }
    }
  }
}

PLUGINLIB_EXPORT_CLASS(costmap::MapLayer, costmap::Layer)
