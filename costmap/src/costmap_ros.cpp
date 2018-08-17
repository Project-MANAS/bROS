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
      size_x_(10000.0),
      size_y_(10000.0),
      resolution_(1.0),
      min_freq_(1.0),
      freq_(min_freq_),
      ros_clock_(RCL_ROS_TIME),
      plugin_loader_("costmap", "costmap::Layer"),
      plugins_list_("costmap::MapLayer, costmap::ObstacleLayer, costmap::InflationLayer"),
      buffer_(clock),
      map_update_thread_shutdown_(false)
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

    tf2_ros::TransformListener tfl(buffer_);
    rclcpp::Rate rate(1.0);
    std::string tf_error;
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

    compute_freq_thread_ = std::thread(&CostmapROS::computeFreqLoop, this);
    last_publish_.sec = 0;
    map_update_thread_ = std::thread(&CostmapROS::mapUpdateLoop, this);
  }

  CostmapROS::~CostmapROS()
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_.join();
    compute_freq_thread_.join();
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

  void CostmapROS::mapUpdateLoop(){
    double publish_cycle = 1.0 / freq_;
    rclcpp::Rate rate(freq_);
    builtin_interfaces::msg::Time current;
    while(rclcpp::ok()){
      if(!map_update_thread_shutdown_) {
        rclcpp::Rate rate(freq_);
        builtin_interfaces::msg::Time start = ros_clock_.now();
        if (freq_ <= 0.0) {
          RCLCPP_WARN(this->get_logger(), "Update frequency is set to %f. Skipping costmap update", freq_);
          return;
        }
        mapUpdate();
        current = ros_clock_.now();
        if (last_publish_.sec + publish_cycle < current.sec) { ;
        }
        rate.sleep();
        builtin_interfaces::msg::Time end = ros_clock_.now();
        double time_taken = (end.nanosec - start.nanosec)/1e9;
        if (time_taken > 1.0 / min_freq_) {
          RCLCPP_WARN(this->get_logger(), "Costmap update loop failed. Desired frequency is %fHz."
              "The loop actually took %f seconds", freq_, time_taken);
        }
      }
      else{
        RCLCPP_INFO(this->get_logger(), "Shutting down costmap...")
      }
    }
  }

  void CostmapROS::mapUpdate(){

  }

  void CostmapROS::computeFreqLoop(){
    rclcpp::Rate rate(1.0);
    while(rclcpp::ok()){
      rate.sleep();
    }
  }
}
