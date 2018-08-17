//
// Created by shivesh on 6/30/18.
//

#include "costmap/costmap.h"

namespace costmap
{
  Costmap::Costmap(
      std::string global_frame_, std::string base_frame, unsigned int size_x, unsigned int size_y, double resolution)
  {
    costmap_ = new unsigned char[size_x * size_y];
  }
  Costmap::~Costmap()
  {
    delete[] costmap_;
  }
}
