#ifndef OBSERVATION_BUFFER_H_
#define OBSERVATION_BUFFER_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <boost/thread.hpp>

namespace costmap {

class Observation {
 public:
  Observation() :
      cloud_(new sensor_msgs::msg::PointCloud2()),
      obstacle_range_(0.0),
      raytrace_range_(0.0) {}

  Observation(geometry_msgs::msg::Point &origin,
              const sensor_msgs::msg::PointCloud2 &cloud,
              double obstacle_range,
              double raytrace_range) :
      origin_(origin),
      cloud_(new sensor_msgs::msg::PointCloud2(cloud)),
      obstacle_range_(obstacle_range),
      raytrace_range_(raytrace_range) {}

  Observation(const Observation &obs) : origin_(obs.origin_),
                                        cloud_(new sensor_msgs::msg::PointCloud2(*(obs.cloud_))),
                                        obstacle_range_(obs.obstacle_range_),
                                        raytrace_range_(obs.raytrace_range_) {}

  Observation(const sensor_msgs::msg::PointCloud2 &cloud, double obstacle_range) :
      cloud_(new sensor_msgs::msg::PointCloud2(cloud)),
      obstacle_range_(obstacle_range),
      raytrace_range_(0.0) {}

  ~Observation() { delete cloud_; }

  geometry_msgs::msg::Point origin_;
  sensor_msgs::msg::PointCloud2 *cloud_;
  double obstacle_range_, raytrace_range_;
 private:
};

class ObservationBuffer {
 public:
  ObservationBuffer(std::string topic_name, int observation_keep_time, int expected_update_rate,
                    double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                    double raytrace_range, tf2_ros::Buffer &tf_buffer, std::string global_frame,
                    std::string sensor_frame, rclcpp::Clock *ros_clock);

  ~ObservationBuffer();

  void bufferCloud(const sensor_msgs::msg::PointCloud2 &cloud);

  void getObservations(std::vector<Observation> &observations);

  bool isCurrent() const;

  inline void lock() {
    lock_.lock();
  }

  inline void unlock() {
    lock_.unlock();
  }

  void resetLastUpdated();

 private:
  void purgeOldObservations();

  std::string topic_name_;
  rclcpp::Duration observation_keep_time_, expected_update_rate_;
  double min_obstacle_height_, max_obstacle_height_, obstacle_range_, raytrace_range_;
  tf2_ros::Buffer &tf_buffer_;
  std::string global_frame_, sensor_frame_;
  rclcpp::Clock *ros_clock_;
  rclcpp::Time last_updated_;
  std::list<Observation> observation_list_;
  boost::recursive_mutex lock_;
};
}
#endif
