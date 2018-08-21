//
// Created by shivesh on 6/30/18.
//

#ifndef COSTMAP_COSTMAP_H_
#define COSTMAP_COSTMAP_H_

#include <string>
#include <memory>
#include <vector>

#include "costmap/map_cell.h"
#include "costmap/layer.h"
namespace costmap
{
class Costmap
{
 public:
  Costmap(std::string global_frame, std::string base_frame, unsigned int size_x, unsigned int size_y,
      double resolution, unsigned char default_char);
  virtual ~Costmap();
  void update(double x, double y, double yaw, bool rolling_window);
  void loadPlugin(std::shared_ptr<Layer> plugin);

  std::string global_frame_, base_frame_;
  unsigned int minx_, miny_, maxx_, maxy_;
  unsigned int origin_x_, origin_y_;
  double resolution_;
  bool rolling_window_;

  MapCell* map_cell;

 private:
    void updateOrigin(double x, double y, double yaw);

    std::vector<std::shared_ptr<Layer>> plugins_;
};
}

#endif //COSTMAP_COSTMAP_H_
