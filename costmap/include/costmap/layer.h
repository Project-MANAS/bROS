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
    void updateBounds(unsigned int origin_x, unsigned int origin_y, double yaw,
                      unsigned int* minx, unsigned int* maxx, unsigned int* miny, unsigned int* maxy);
    void updateCosts(double minx_, double maxx_, double miny_, double maxy_);
  };
}

#endif //COSTMAP_LAYER_H_
