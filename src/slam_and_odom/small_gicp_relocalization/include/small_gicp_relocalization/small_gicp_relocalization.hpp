// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);
  ~SmallGicpRelocalizationNode();

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void initializeGlobalMap();
  void performRegistration();
  void performGlobalSearch();
  struct StartupCandidate
  {
    Eigen::Isometry3d pose;
    double score;
    int sequence;
  };
  bool acceptStartupCandidate(
    const Eigen::Isometry3d & candidate_pose, double candidate_score,
    Eigen::Isometry3d & accepted_pose, double & accepted_score, int & matched_sequence);
  bool isStartupCandidateConsistent(
    const StartupCandidate & history, const Eigen::Isometry3d & candidate_pose,
    double candidate_score) const;
  std::vector<double> buildStartupYawCandidates(int yaw_samples) const;
  double getYawFromPose(const Eigen::Isometry3d & pose) const;
  double normalizeAngle(double angle) const;
  void clearStartupCandidates();
  // 把实时帧点云投回原始地图系，裁掉赛场外的人群点（赛场内全留，场外只留天花板）
  void cropCourtCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;

  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  float tf_prediction_offset_;
  std::vector<double> init_pose_;

  double global_search_step_;
  int global_search_yaw_samples_;
  int global_search_candidate_iters_;

  double global_search_coarse_step_;
  int global_search_coarse_yaw_samples_;
  float global_search_coarse_leaf_size_;
  int global_search_coarse_iters_;
  int global_search_fine_iters_;

  double map_filter_x_min_;
  double map_filter_x_max_;
  double map_filter_y_min_;
  double map_filter_y_max_;
  double map_filter_z_min_;
  double map_filter_z_max_;

  double relocalization_map_filter_x_min_;
  double relocalization_map_filter_x_max_;
  double relocalization_map_filter_y_min_;
  double relocalization_map_filter_y_max_;
  double relocalization_global_search_coarse_step_;

  double continuous_update_rate_;
  double update_min_translation_;
  double update_min_rotation_;

  double max_divergence_speed_;
  double max_z_deviation_;

  bool enable_global_search_;
  bool startup_consistency_enabled_;
  double startup_consistency_trans_thresh_;
  double startup_consistency_yaw_thresh_;
  double startup_consistency_score_ratio_;
  int startup_candidate_max_count_;
  bool startup_yaw_prior_enabled_;
  double startup_yaw_prior_;
  double startup_yaw_prior_tolerance_;
  int startup_candidate_sequence_{0};
  std::vector<StartupCandidate> startup_candidates_;

  // 赛场实时裁剪：在原始地图系下，赛场框内全保留，框外只留 Z>court_crop_z_min_ 的天花板点
  bool enable_court_crop_;
  double court_crop_x_min_;
  double court_crop_x_max_;
  double court_crop_y_min_;
  double court_crop_y_max_;
  double court_crop_margin_;
  double court_crop_z_min_;
  // T_flip = T_{base_frame <- lidar_frame}，initializeGlobalMap 用它预乘地图；裁剪时用其逆把点投回原始地图系
  Eigen::Isometry3d map_flip_tf_{Eigen::Isometry3d::Identity()};

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string odom_topic_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  Eigen::Isometry3d last_good_odom_to_base_link_{Eigen::Isometry3d::Identity()};

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  std::mutex cloud_mutex_;
  std::mutex pose_mutex_;
  std::mutex startup_candidate_mutex_;
  std::atomic<bool> global_map_initialized_{false};
  std::atomic<bool> is_registering_{false};
  std::atomic<bool> global_search_done_{false};
  std::atomic<bool> use_relocalization_search_range_{false};
  std::thread registration_thread_;
  std::atomic<bool> run_thread_{true};
  int lost_tracking_count_{0};

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
