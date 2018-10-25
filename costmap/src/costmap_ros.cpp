//
// Created by shivesh on 29/6/18.
//

#include "costmap/costmap_ros.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace costmap {
CostmapROS::CostmapROS() :
    Node("costmap_ros"),
    global_frame_("map"),
    base_frame_("base_link"),
    size_x_(200),
    size_y_(200),
    resolution_(0.5),
    cells_x_(0),
    cells_y_(0),
    default_cost_(0),
    rolling_window_(true),
    buffer_(duration),
    tfl(buffer_),
    plugin_loader_("costmap", "costmap::Layer"),
    plugins_list_("costmap::MapLayer"),
    ros_clock_(RCL_ROS_TIME),
    min_update_freq_(2.0),
    update_freq_(min_update_freq_),
    updated_(false),
    transform_tolerance_(1.0),
    min_publish_freq_(1.0),
    publish_freq_(min_publish_freq_),
    map_publish_thread_shutdown_(false),
    odom_topic_("/odom"),
    vel_init(false),
    pub_topic_("/costmap") {
  cells_x_ = (unsigned int) (size_x_ / resolution_);
  cells_y_ = (unsigned int) (size_y_ / resolution_);
  costmap_ = new Costmap(global_frame_, base_frame_, cells_x_, cells_y_, resolution_, default_cost_);

  std::string type;
  for (char i : plugins_list_) {
    if (i == ',') {
      pluginLoader(type);
      type = std::string("");
      continue;
    }
    if (i == ' ')
      continue;
    type += i;
  }
  pluginLoader(type);

  rclcpp::Rate rate(1.0);
  while (rclcpp::ok()) {
    try {
      rclcpp::Time time = rclcpp::Time(0);
      tf2::TimePoint tf2_time(std::chrono::seconds(time.nanoseconds()));
      buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      break;
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rate.sleep();
    }
  }

  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_,
                                                                     std::bind(&CostmapROS::velocityCallback,
                                                                               this, _1));
  publisher_ = new CostmapPublisher(pub_topic_);
  compute_freq_thread_ = std::thread(&CostmapROS::computeFreqLoop, this);
  map_update_thread_ = std::thread(&CostmapROS::mapUpdateLoop, this);
  map_publish_thread_ = std::thread(&CostmapROS::mapPublishLoop, this);
}

CostmapROS::~CostmapROS() {
  delete costmap_;
  delete publisher_;
  map_publish_thread_shutdown_ = true;
  map_update_thread_.join();
  map_publish_thread_.join();
  compute_freq_thread_.join();
}

void CostmapROS::pluginLoader(std::string type) {
  RCLCPP_INFO(this->get_logger(), "Loading class %s", type.c_str());
  try {
    std::cout << "test1" << std::endl;
    std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(type);
    costmap_->loadPlugin(plugin);
    plugin->initialise(global_frame_, cells_x_, cells_y_, cells_x_ / 2, cells_y_ / 2, resolution_, rolling_window_);
  }
  catch (pluginlib::LibraryLoadException &e) {
    RCLCPP_ERROR(this->get_logger(), "Class %s does not exist", type.c_str());
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Could not load class %s", type.c_str());
  }
}

void CostmapROS::mapUpdateLoop() {
  rclcpp::Rate rate(update_freq_);
  rclcpp::Time current;
  updated_ = false;
  while (rclcpp::ok()) {
    rclcpp::Rate rate(update_freq_);
    rclcpp::Time start = ros_clock_.now();
    if (update_freq_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Costmap update frequency is set to %f. Skipping update loop",
                  min_publish_freq_);
      return;
    }
    current = ros_clock_.now();
    mapUpdate();
    updated_ = true;
    rclcpp::Time finish = ros_clock_.now();
    rate.sleep();
    double time_taken = (finish.nanoseconds() - start.nanoseconds()) / 1e9;
    if (time_taken > 1.0 / min_update_freq_) {
      RCLCPP_WARN(this->get_logger(),
                  "Costmap update loop failed. Desired frequency is %fHz. The loop actually took %f seconds",
                  update_freq_, time_taken);
    }
  }
}

void CostmapROS::mapUpdate() {
  geometry_msgs::msg::PoseStamped pose;
  pose.pose = odom_.pose.pose;
  if (getRobotPose(pose)) {
    costmap_->update(pose.pose, rolling_window_);
    publisher_->prepareMap(costmap_);
  }
}

bool CostmapROS::getRobotPose(geometry_msgs::msg::PoseStamped &pose) {
  pose.header.frame_id = base_frame_;
  pose.header.stamp = ros_clock_.now();
  rclcpp::Time start = ros_clock_.now();
  try {
    rclcpp::Time time = rclcpp::Time(0);
    tf2::TimePoint tf2_time(std::chrono::nanoseconds(time.nanoseconds()));
    geometry_msgs::msg::TransformStamped tfp = buffer_.lookupTransform("odom", global_frame_, tf2_time);
    tf2::doTransform(pose, pose, tfp);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return false;
  }
  rclcpp::Time finish = ros_clock_.now();
  if (RCL_NS_TO_S(finish.nanoseconds()) - RCL_NS_TO_S(start.nanoseconds()) > transform_tolerance_) {
    RCLCPP_WARN(this->get_logger(),
                "Costmap %s to %s transform timed out. Current time: %d, global_pose stamp %d, tolerance %d",
                global_frame_.c_str(), base_frame_.c_str(), RCL_NS_TO_S(finish.nanoseconds()), pose.header.stamp,
                transform_tolerance_);
  }
  return true;
}

void CostmapROS::mapPublishLoop() {
  rclcpp::Rate rate(publish_freq_);
  rclcpp::Time current;
  while (rclcpp::ok()) {
    if (!map_publish_thread_shutdown_) {
      rclcpp::Rate rate(publish_freq_);
      rclcpp::Time start = ros_clock_.now();
      if (publish_freq_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Costmap publish frequency is set to %f. Skipping publish loop.",
                    min_publish_freq_);
        return;
      }
      current = ros_clock_.now();
      while (!updated_) {
        RCLCPP_WARN(this->get_logger(), "Costmap publish loop failed. Desired frequency is %fHz.",
                    min_publish_freq_);
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
      publisher_->publish();
      updated_ = false;

      rclcpp::Time finish = ros_clock_.now();
      rate.sleep();
      double time_taken = (finish.nanoseconds() - start.nanoseconds()) / 1e9;
      if (time_taken > 1.0 / min_publish_freq_) {
        RCLCPP_WARN(this->get_logger(),
                    "Costmap publish loop failed. Desired frequency is %fHz. The loop actually took %f seconds",
                    publish_freq_,
                    time_taken);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Shutting down costmap...");
      break;
    }
  }
}

void CostmapROS::computeFreqLoop() {
  rclcpp::Rate rate(1.0);
  while (rclcpp::ok()) {
    if (vel_init) {
      double linear = sqrt(pow(odom_.twist.twist.linear.x, 2) + pow(odom_.twist.twist.linear.y, 2)
                               + pow(odom_.twist.twist.linear.z, 2));
      double angular = sqrt(pow(odom_.twist.twist.angular.x, 2) + pow(odom_.twist.twist.angular.y, 2)
                                + pow(odom_.twist.twist.angular.z, 2));
      //TODO
    }
    rate.sleep();
  }
}

void CostmapROS::velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = *msg;
  vel_init = true;
}
}
