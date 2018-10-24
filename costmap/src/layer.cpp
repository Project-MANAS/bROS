//
// Created by shivesh on 7/1/18.
//

#include "costmap/layer.h"

namespace costmap {
Layer::Layer() {

}

Layer::~Layer() {

}

void Layer::initialise(std::string global_frame,
                       unsigned int size_x,
                       unsigned int size_y,
                       unsigned int origin_x,
                       unsigned int origin_y,
                       double resolution,
                       bool rolling_window) {

}

void Layer::updateBounds(double *minx, double *maxx, double *miny, double *maxy) {
}

void Layer::updateCosts(MapCell *mc, double *minx, double *maxx, double *miny, double *maxy) {
}


void Layer::touch(double x, double y, double *minx, double *maxx, double *miny, double *maxy) {
  *minx = std::min(x, *minx);
  *maxx = std::max(x, *maxx);
  *miny = std::min(y, *miny);
  *maxy = std::max(y, *maxy);
}
}
