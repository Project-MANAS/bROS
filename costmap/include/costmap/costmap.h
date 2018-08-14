//
// Created by shivesh on 6/30/18.
//

#ifndef COSTMAP_COSTMAP_H_
#define COSTMAP_COSTMAP_H_

#include <string>
namespace costmap
{
class Costmap
{
 public:
  Costmap(std::string global_frame, std::string base_frame, double size_x, double size_y, double resolution);
  virtual ~Costmap();

 protected:
  unsigned char* costmap_;
};
}

#endif //COSTMAP_COSTMAP_H_
