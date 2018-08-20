

//
// Created by shivesh on 6/30/18.
//

#include "costmap/costmap.h"

namespace costmap
{
  Costmap::Costmap(std::string global_frame, std::string base_frame,
          unsigned int size_x, unsigned int size_y, double resolution) :
          global_frame_(global_frame),
          base_frame_(base_frame),
          size_x_(size_x),
          size_y_(size_y),
          origin_x_(0.0),
          origin_y_(0.0)
  {
    int size = int(size_x / resolution) * int(size_y / resolution);
    costmap_ = new unsigned char[size];
  }

  Costmap::~Costmap()
  {
    while(plugins_.size() > 0)
      plugins_.pop_back();
    delete[] costmap_;
  }

  void Costmap::loadPlugin(std::shared_ptr<Layer> plugin){
    plugins_.push_back(plugin);
  };

  void Costmap::updateOrigin(double x, double y, double yaw){
    //TODO
  }

  void Costmap::update(double x, double y, double yaw, bool rolling_window_){
    if(costmap_)
      updateOrigin(x, y, yaw);
    double minx_ = 1e30, miny_ = 1e30;
    double maxx_ = 1e-30, maxy_ = -1e30;

    for(std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin){
      (*plugin)->updateBounds(x, y, yaw, &minx_, &maxx_, &miny_, &maxy_);
    }

    for(std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin){
      (*plugin)->updateCosts(minx_, maxx_, miny_, maxy_);
    }
  }
}
