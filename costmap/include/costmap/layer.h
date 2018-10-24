//
// Created by shivesh on 7/1/18.
//

#ifndef COSTMAP_LAYER_H_
#define COSTMAP_LAYER_H_

#include "costmap/map_cell.h"
#include "brosdb.h"
#include <string>

namespace costmap {
class Layer {
 public:
  Layer();

  virtual ~Layer();

  virtual void initialise(std::string global_frame,
                          unsigned int size_x,
                          unsigned int size_y,
                          unsigned int origin_x,
                          unsigned int origin_y,
                          double resolution,
                          bool rolling_window);

  virtual void updateBounds(double *minx, double *maxx, double *miny, double *maxy);

  virtual void
  updateCosts(MapCell *mc, double *minx, double *maxx, double *miny, double *maxy);

  inline void mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
  }

  inline bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const {
    if (wx < origin_x_ || wy < origin_y_)
      return false;

    mx = (int) ((wx - origin_x_) / resolution_);
    my = (int) ((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
      return true;

    return false;
  }

  inline double gridsToMetres(double gridLength) const {
    return (gridLength - 0.5) * resolution_;
  }
  void touch(double x, double y, double *minx, double *maxx, double *miny, double *maxy);

  unsigned int minx_, maxx_, miny_, maxy_;
  int origin_x_, origin_y_;
  unsigned int size_x_, size_y_;
  double resolution_;
  bool map_received_, rolling_window_;
  MapCell *map_;

};
}

#endif //COSTMAP_LAYER_H_
