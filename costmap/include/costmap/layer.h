//
// Created by shivesh on 7/1/18.
//

#ifndef COSTMAP_LAYER_H_
#define COSTMAP_LAYER_H_

namespace costmap
{
  class Layer
  {
   public:
    Layer();
    void updateBounds(double x, double y, double yaw, double* minx_, double* maxx_, double* miny_, double* maxy_);
    void updateCosts(double minx_, double maxx_, double miny_, double maxy_);
  };
}

#endif //COSTMAP_LAYER_H_
