//
// Created by shivesh on 20/8/18.
//

#ifndef COSTMAP_COSTMAP_PUBLISHER_H_
#define COSTMAP_COSTMAP_PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <costmap/costmap.h>

namespace costmap{
 class CostmapPublisher : public rclcpp::Node
 {
  public:
   CostmapPublisher(std::string topic);
   ~CostmapPublisher();
   void publish();
   void prepareMap(Costmap* costmap);

  private:
   std::string topic_;
   char* costmap_translation_table_;
   nav_msgs::msg::OccupancyGrid map_;
   rclcpp::Clock ros_clock_;
   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

 };
}

#endif //COSTMAP_COSTMAP_PUBLISHER_H_
