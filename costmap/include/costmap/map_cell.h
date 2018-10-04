//
// Created by shivesh on 8/16/18.
//

#ifndef COSTMAP_MAP_CELL_H_
#define COSTMAP_MAP_CELL_H_

namespace costmap {
class MapCell {
 public:
  MapCell();

  MapCell(const MapCell &mc);

  unsigned int x, y;
  unsigned char cost;
};
}

#endif //COSTMAP_MAP_CELL_H_
