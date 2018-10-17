//
// Created by shivesh on 7/1/18.
//

#ifndef COSTMAP_LAYER_H_
#define COSTMAP_LAYER_H_

#include "costmap/map_cell.h"
#include <string>

namespace costmap {
class Layer {
 public:
  Layer();

  virtual ~Layer();

  virtual void initialise(std::string global_frame, unsigned int size_x, unsigned int size_y, unsigned int origin_x, unsigned int origin_y,
                          double resolution, bool rolling_window);

  virtual void updateBounds(unsigned int *minx, unsigned int *maxx, unsigned int *miny, unsigned int *maxy);

  virtual void
  updateCosts(MapCell *mc, unsigned int minx, unsigned int maxx, unsigned int miny, unsigned int maxy);
};
}

#endif //COSTMAP_LAYER_H_
