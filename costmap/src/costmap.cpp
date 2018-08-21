

//
// Created by shivesh on 6/30/18.
//

#include "costmap/costmap.h"

namespace costmap
{
  Costmap::Costmap(std::string global_frame, std::string base_frame,
          unsigned int size_x, unsigned int size_y, double resolution, unsigned char default_cost) :
          global_frame_(global_frame),
          base_frame_(base_frame),
          minx_(0),
          miny_(0),
          maxx_(1e10),
          maxy_(1e10),
          origin_x_(5e9),
          origin_y_(5e9),
          resolution_(resolution)
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

  void Costmap::updateOrigin(double x, double y, double yaw){
    origin_x_ = 5e9 - x / resolution_;
    origin_y_ = 5e9 - y / resolution_;
    //TODO yaw
  }

  void Costmap::update(double x, double y, double yaw, bool rolling_window){
    if(rolling_window)
      updateOrigin(x, y, yaw);

    std::vector<std::shared_ptr<Layer>>::iterator plugin;

    for(plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin){
      (*plugin)->updateBounds(origin_x_, origin_y_, yaw, &minx_, &maxx_, &miny_, &maxy_);
    }

    for(plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin){
      (*plugin)->updateCosts(minx_, maxx_, miny_, maxy_);
    }
  }
}
