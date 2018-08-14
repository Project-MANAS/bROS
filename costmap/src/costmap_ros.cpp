//
// Created by shivesh on 29/6/18.
//

#include "costmap/costmap_ros.h"

using namespace std::chrono_literals;

namespace costmap
{
  CostmapROS::CostmapROS() :
      Node("costmap_node"),
      global_frame_("map"),
      base_frame_("base_link"),
      size_x_(0.0),
      size_y_(0.0),
      resolution_(1.0),
      plugin_loader_("costmap", "costmap::Layer"),
      plugins_list_("costmap::MapLayer, costmap::ObstacleLayer, costmap::InflationLayer")
  {
    costmap_ = new Costmap(global_frame_, base_frame_, size_x_, size_y_, resolution_);

    std::string type;
    for(unsigned int i = 0; i < plugins_list_.length(); i++)
    {
      if(plugins_list_[i] == ',')
      {
        pluginLoader(type);
        type = std::string("");
        continue;
      }
      if(plugins_list_[i] == ' ')
        continue;
      type += plugins_list_[i];
    }
    pluginLoader(type);

//    tf2_ros::TransformListener tfl(buffer_);
//    rclcpp::Rate rate(1.0);
//    std::string tf_error;
//    while(rclcpp::ok())
//    {
//      try
//      {
//        rclcpp::Time time = rclcpp::Time(0);
//        tf2::TimePoint tf2_time(std::chrono::nanoseconds(time.nanoseconds()));
//        buffer_.lookupTransform(global_frame_, base_frame_, tf2_time);
//        break;
//      }
//      catch (tf2::TransformException &ex)
//      {
//        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
//        rate.sleep();
//      }
//    }
  }

  CostmapROS::~CostmapROS()
  {

  }

  void CostmapROS::pluginLoader(std::string type)
  {
    RCLCPP_INFO(this->get_logger(), "Loading class %s", type.c_str());
    try {
      auto plugin = plugin_loader_.createSharedInstance(type);
    }
    catch (pluginlib::LibraryLoadException e) {
      RCLCPP_ERROR(this->get_logger(), "Class %s does not exist", type.c_str());
    }
    catch (...){
      RCLCPP_ERROR(this->get_logger(), "Could not load class %s", type.c_str());
    }
  }
}
