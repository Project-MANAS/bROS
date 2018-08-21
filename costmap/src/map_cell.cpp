//
// Created by shivesh on 8/16/18.
//

#include "costmap/map_cell.h"

namespace costmap{
  MapCell::MapCell() : x(0), y(0), cost(0)
  {}

  MapCell::MapCell(const MapCell& mc) : x(mc.x), y(mc.y), cost(mc.cost)
  {}
}