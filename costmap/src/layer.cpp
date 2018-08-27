//
// Created by shivesh on 7/1/18.
//

#include "costmap/layer.h"

namespace costmap
{
  Layer::Layer()
  {

  }

  Layer::~Layer()
  {

  }
  void Layer::initialise(unsigned int size_x, unsigned int size_y, unsigned int origin_x, unsigned int origin_y, double resolution){

  }

  void Layer::updateBounds(unsigned int* minx, unsigned int* maxx, unsigned int* miny, unsigned int* maxy,
      double* origin_x, double* origin_y, bool rolling_window){
  }

  void Layer::updateCosts(MapCell* mc, unsigned int minx, unsigned int maxx, unsigned int miny, unsigned int maxy){

  }
}
