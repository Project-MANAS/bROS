//
// Created by squadrick on 9/23/18.
//

#include "costmap/observation_buffer.h"

namespace costmap {
ObservationBuffer::ObservationBuffer(std::string topic_name, int observation_keep_time, int expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, tf2_ros::Buffer &tf_buffer, std::string global_frame,
                                     std::string sensor_frame, rclcpp::Clock *ros_clock) :
    observation_keep_time_(observation_keep_time, 0),
    expected_update_rate_(expected_update_rate, 0),
    min_obstacle_height_(min_obstacle_height),
    max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range),
    raytrace_range_(raytrace_range),
    tf_buffer_(tf_buffer),
    global_frame_(global_frame),
    sensor_frame_(sensor_frame) {
  ros_clock_ = ros_clock;
}

ObservationBuffer::~ObservationBuffer() {
  ros_clock_ = nullptr;
}

void ObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2 &cloud) {
  geometry_msgs::msg::PointStamped global_origin;

  observation_list_.push_front(Observation());
  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  {
    geometry_msgs::msg::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;

    tf_buffer_.transform(local_origin, global_origin, global_frame_);
    tf2::convert(global_origin.point, observation_list_.front().origin_);

    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;

    sensor_msgs::msg::PointCloud2 global_frame_cloud;

    tf_buffer_.transform(cloud, global_frame_cloud, global_frame_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    sensor_msgs::msg::PointCloud2 &observation_cloud = *(observation_list_.front().cloud_);
    observation_cloud.height = global_frame_cloud.height;
    observation_cloud.width = global_frame_cloud.width;
    observation_cloud.fields = global_frame_cloud.fields;
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    observation_cloud.point_step = global_frame_cloud.point_step;
    observation_cloud.row_step = global_frame_cloud.row_step;
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    unsigned int cloud_size = global_frame_cloud.height * global_frame_cloud.width;

    sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
    modifier.resize(cloud_size);

    unsigned int point_count = 0;

    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");

    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();

    for (auto iter_global = global_frame_cloud.data.begin(); iter_global != global_frame_cloud.data.end();
         ++iter_z, iter_global += global_frame_cloud.point_step) {
      if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_) {
        std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
        iter_obs += global_frame_cloud.point_step;
        ++point_count;
      }
    }

    modifier.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  }
//  catch (TransformException &ex) {
//    observation_list_.pop_front();
//    return;
//  }
  this->resetLastUpdated();
  this->purgeOldObservations();
}

void ObservationBuffer::getObservations(std::vector<Observation> &observations) {
  purgeOldObservations();

  for (auto obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    observations.push_back(*obs_it);
}

void ObservationBuffer::purgeOldObservations() {
  if (observation_list_.empty())
    return;

  for (auto obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
    if ((last_updated_ - obs_it->cloud_->header.stamp) > observation_keep_time_) {
      observation_list_.erase(obs_it, observation_list_.end());
      return;
    }
  }
}

bool ObservationBuffer::isCurrent() const {
  if (expected_update_rate_.nanoseconds() == 0)
    return true;

  auto time_diff = ros_clock_->now() - last_updated_;

  bool current = time_diff <= expected_update_rate_;
  if (!current)
    /*
     * WARNING
     * Can't use RCLCPP_INFO since this is not a Node
     * Might have to pass an instance of `this->get_logger()`
     * This is where we'll have to use topic_name_
     * RCLCPP_INFO(this->get_logger(), "The observation buffer has not been updates for %s seconds.", time_diff.nanoseconds());
     */
    return current;
}

void ObservationBuffer::resetLastUpdated() {
  last_updated_ = ros_clock_->now();
}
}