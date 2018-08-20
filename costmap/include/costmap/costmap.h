//
// Created by shivesh on 6/30/18.
//

#ifndef COSTMAP_COSTMAP_H_
#define COSTMAP_COSTMAP_H_

#include <string>
#include <memory>
#include <vector>

#include "costmap/layer.h"
namespace costmap
{
class Costmap
{
 public:
  Costmap(std::string global_frame, std::string base_frame, unsigned int size_x, unsigned int size_y, double resolution);
  virtual ~Costmap();
  void update(double x, double y, double yaw, bool rolling_window);
  void loadPlugin(std::shared_ptr<Layer> plugin);
  bool isRolling(){
    return rolling_window_;
  }

 protected:
  unsigned char* costmap_;

 private:
    void updateOrigin(double x, double y, double yaw);

    std::string global_frame_, base_frame_;
    unsigned int size_x_, size_y_;
    double origin_x_, origin_y_, resolution;
    double size;
    bool rolling_window_;

    std::vector<std::shared_ptr<Layer>> plugins_;
};
}

#endif //COSTMAP_COSTMAP_H_
