//
// Created by shivesh on 20/8/18.
//

#ifndef COSTMAP_COSTMAP_PUBLISHER_H_
#define COSTMAP_COSTMAP_PUBLISHER_H_

namespace costmap{
 class CostmapPublisher{
  public:
   CostmapPublisher();
   ~CostmapPublisher();
   void publish();

  private:
   void prepareMap();
 };
}

#endif //COSTMAP_COSTMAP_PUBLISHER_H_
