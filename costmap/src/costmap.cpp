

//
// Created by shivesh on 6/30/18.
//

#include "costmap/costmap.h"

namespace costmap
{
  Costmap::Costmap(std::string global_frame, std::string base_frame,
          unsigned int size_x, unsigned int size_y, double resolution, unsigned char default_cost) :
          Node("costmap"),
          global_frame_(global_frame),
          base_frame_(base_frame),
          minx_(0),
          miny_(0),
          maxx_(size_x),
          maxy_(size_y),
          origin_x_(size_x/2),
          origin_y_(size_y/2),
          size_x_(size_x),
          size_y_(size_y),
          resolution_(resolution),
          origin_init_(false)
  {
    int size = size_x * size_y;
    map_cell = new MapCell[size];
    int k = 0;
    for(unsigned int i = 0; i < size_x; ++i){
      for(unsigned int j = 0; j < size_y; ++j){
        map_cell[k].x = j;
        map_cell[k].y = i;
        map_cell[k].cost = default_cost;
        ++k;
      }
    }
  }

  Costmap::~Costmap()
  {
    while(plugins_.size() > 0)
      plugins_.pop_back();
    delete map_cell;
  }

  void Costmap::loadPlugin(std::shared_ptr<Layer> plugin){
    plugins_.push_back(plugin);
  }

  void Costmap::update(const geometry_msgs::msg::Pose& origin, bool rolling_window){
    std::vector<std::shared_ptr<Layer>>::iterator plugin;
    for(plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin){
      (*plugin)->updateBounds(&minx_, &maxx_, &miny_, &maxy_, rolling_window);
    }

    for(plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin){
      (*plugin)->updateCosts(map_cell, minx_, maxx_, miny_, maxy_);
    }

    if(rolling_window || !origin_init_){
      origin_init_ = true;
      map_origin_ = origin;
      double origin_x = origin.position.x - (size_x_ * resolution_ / 2);
      double origin_y = origin.position.y - (size_y_ * resolution_ / 2);
      map_origin_.position.x = origin_x;
      map_origin_.position.y = origin_y;
    }
    else{
      map_origin_.position.x = ((double)minx_-origin_x_)*resolution_;
      map_origin_.position.y = ((double)miny_-origin_y_)*resolution_;
    }
    RCLCPP_INFO(this->get_logger(),"%f %f", map_origin_.position.x, map_origin_.position.y);
  }
}
