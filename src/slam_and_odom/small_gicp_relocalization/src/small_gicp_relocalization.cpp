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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Eigenvalues>

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 2);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 1.0);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 4.0);
  this->declare_parameter("tf_prediction_offset", 0.2);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  this->declare_parameter("global_search_step", 2.0);
  this->declare_parameter("global_search_yaw_samples", 6);
  this->declare_parameter("global_search_candidate_iters", 5);

  this->declare_parameter("global_search_coarse_step", 4.0);
  this->declare_parameter("global_search_coarse_yaw_samples", 4);
  this->declare_parameter("global_search_coarse_leaf_size", 1.0);
  this->declare_parameter("global_search_coarse_iters", 2);
  this->declare_parameter("global_search_fine_iters", 15);

  this->declare_parameter("map_filter_x_min", -5.0);
  this->declare_parameter("map_filter_x_max", 5.0);
  this->declare_parameter("map_filter_y_min", -5.0);
  this->declare_parameter("map_filter_y_max", 5.0);
  this->declare_parameter("map_filter_z_min", -1.0);
  this->declare_parameter("map_filter_z_max", 8.0);

  this->declare_parameter("continuous_update_rate", 0.5);
  this->declare_parameter("update_min_translation", 0.03);
  this->declare_parameter("update_min_rotation", 0.02);

  this->declare_parameter("enable_global_search", false);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("tf_prediction_offset", tf_prediction_offset_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);

  this->get_parameter("global_search_step", global_search_step_);
  this->get_parameter("global_search_yaw_samples", global_search_yaw_samples_);
  this->get_parameter("global_search_candidate_iters", global_search_candidate_iters_);

  this->get_parameter("global_search_coarse_step", global_search_coarse_step_);
  this->get_parameter("global_search_coarse_yaw_samples", global_search_coarse_yaw_samples_);
  this->get_parameter("global_search_coarse_leaf_size", global_search_coarse_leaf_size_);
  this->get_parameter("global_search_coarse_iters", global_search_coarse_iters_);
  this->get_parameter("global_search_fine_iters", global_search_fine_iters_);

  this->get_parameter("map_filter_x_min", map_filter_x_min_);
  this->get_parameter("map_filter_x_max", map_filter_x_max_);
  this->get_parameter("map_filter_y_min", map_filter_y_min_);
  this->get_parameter("map_filter_y_max", map_filter_y_max_);
  this->get_parameter("map_filter_z_min", map_filter_z_min_);
  this->get_parameter("map_filter_z_max", map_filter_z_max_);

  this->get_parameter("continuous_update_rate", continuous_update_rate_);
  this->get_parameter("update_min_translation", update_min_translation_);
  this->get_parameter("update_min_rotation", update_min_rotation_);

  this->get_parameter("enable_global_search", enable_global_search_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // 用一个独立的轻量级线程来跑计算，不占 ROS 线程
  registration_thread_ = std::thread([this]() {
    while (run_thread_ && rclcpp::ok()) {
      if (!global_search_done_) {
        this->performGlobalSearch();
      } else {
        this->performRegistration();
      }
      int sleep_ms = static_cast<int>(1000.0 / std::max(0.01, continuous_update_rate_));
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
  });

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz 始终发布 TF
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));

  init_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SmallGicpRelocalizationNode::initializeGlobalMap, this));
}

SmallGicpRelocalizationNode::~SmallGicpRelocalizationNode()
{
  run_thread_ = false;
  if (registration_thread_.joinable()) {
    registration_thread_.join();
  }
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());
}

void SmallGicpRelocalizationNode::initializeGlobalMap()
{
  if (global_map_initialized_) {
    return;
  }

  Eigen::Affine3d odom_to_lidar_odom;
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      base_frame_, lidar_frame_, tf2::TimePointZero);
    odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
    RCLCPP_INFO_STREAM(
      this->get_logger(), "odom_to_lidar_odom: translation = "
                            << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                            << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Initial TF lookup failed: %s. Retrying...", ex.what());
    return;
  }

  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  global_map_initialized_ = true;
  init_timer_->cancel();
  init_timer_.reset();
  RCLCPP_INFO(this->get_logger(), "Global map initialized and KdTree built successfully.");
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (!global_map_initialized_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_process(new pcl::PointCloud<pcl::PointXYZ>());
  
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
      return;
    }
    // 极速交换指针 (O(1))，而不是拷贝整个点云 (O(N))
    std::swap(accumulated_cloud_, cloud_to_process);
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *cloud_to_process, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;      // 缩减迭代次数减少 CPU 压力

  Eigen::Isometry3d initial_guess;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    initial_guess = previous_result_t_;
  }

  auto result = register_->align(*target_, *source_, *target_tree_, initial_guess);

  if (result.converged && result.num_inliers > 0) {
    // 连续 GICP 会因为单帧点云缺乏约束（比如在走廊里）而产生滑动（也就是你说的“飘走了”）
    // 一旦全局重定位成功，point_lio 的 odom 已经非常精准，不需要高频 GICP 强行纠正。
    // 如果后续需要修正漂移，建议加长时间间隔或通过协方差判断，这里暂时只打印不更新。
    // std::lock_guard<std::mutex> lock(pose_mutex_);
    // result_t_ = result.T_target_source;
    // previous_result_t_ = result_t_;

    // 计算重定位可信度
    double overlap_ratio = std::min(1.0, (double)result.num_inliers / source_->size());
    double rmse = std::sqrt(result.error / result.num_inliers);
    
    // 退化检测 (Degeneracy Detection)
    // 分析 Hessian 矩阵特征值，如果最小特征值极小，说明在某方向上没有约束力（如长直走廊）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(result.H);
    double min_eigenvalue = eigensolver.eigenvalues().minCoeff();
    double degeneracy_metric = min_eigenvalue / std::max(1.0, (double)result.num_inliers);
    
    // 综合可信度评分 (0~100)
    // 重叠率权重占70%，误差权重占30%（0.5米误差得0分，0误差得满分）
    double overlap_score = overlap_ratio * 100.0;
    double rmse_score = std::max(0.0, 100.0 - (rmse * 200.0));
    double confidence = (overlap_score * 0.7) + (rmse_score * 0.3);

    // 如果出现特征退化，轻度惩罚得分，迫使算法要求更高的重叠率才能采信
    if (degeneracy_metric < 5.0) {
      double penalty = std::max(0.5, degeneracy_metric / 5.0); // 最多扣减一半分数
      confidence *= penalty;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "[Degeneracy Warning] Min Eigenvalue metric (%.3f) too low! Penalizing confidence to %.1f", 
        degeneracy_metric, confidence);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
      "[Confidence Report] Overall Score: %.1f/100  (Overlap: %.1f%%, Avg Error: %.3f meters, Eigen: %.2f)", 
      confidence, overlap_ratio * 100.0, rmse, degeneracy_metric);

    // 恢复连续 GICP 重定位：只有在可信度大于 40 分时才尝试更新地图 TF
    if (confidence > 40.0) {
      lost_tracking_count_ = 0;
      Eigen::Isometry3d new_pose = result.T_target_source;
      double translation_diff = (new_pose.translation() - previous_result_t_.translation()).norm();
      
      Eigen::AngleAxisd angle_axis(new_pose.linear().transpose() * previous_result_t_.linear());
      double angle_diff = std::abs(angle_axis.angle());

      // 死区拦截 (Deadband Interception)
      if (translation_diff > update_min_translation_ || angle_diff > update_min_rotation_) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        result_t_ = new_pose;
        previous_result_t_ = result_t_;
        RCLCPP_INFO(this->get_logger(), "★ ★ GICP 已更正 ★ ★ 平移纠正: %.3f米, 旋转纠正: %.3f弧度", translation_diff, angle_diff);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "GICP 误差极小 (%.3f米), 跳过更正，保持 Point-LIO 丝滑轨迹", translation_diff);
      }
    } else {
      lost_tracking_count_++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "GICP confidence too low (%.1f), skipping continuous TF update to prevent drift.", confidence);
      if (lost_tracking_count_ > 5) {
        RCLCPP_ERROR(this->get_logger(), "Tracking lost for too long (confidence low)! Triggering re-initialization...");
        global_search_done_ = false;
        lost_tracking_count_ = 0;
      }
    }
  } else {
    lost_tracking_count_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "GICP did not converge.");
    if (lost_tracking_count_ > 5) {
      RCLCPP_ERROR(this->get_logger(), "Tracking lost for too long (no converge)! Triggering re-initialization...");
      global_search_done_ = false;
      lost_tracking_count_ = 0;
    }
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (!global_search_done_) {
    return;
  }

  Eigen::Isometry3d current_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose = result_t_;
  }

  if (current_pose.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `tf_prediction_offset_` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(tf_prediction_offset_);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = current_pose.translation();
  const Eigen::Quaterniond rotation(current_pose.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    std::lock_guard<std::mutex> lock(pose_mutex_);
    previous_result_t_ = result_t_ = map_to_odom;
    global_search_done_ = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

void SmallGicpRelocalizationNode::performGlobalSearch()
{
  if (!global_map_initialized_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_process(new pcl::PointCloud<pcl::PointXYZ>());
  
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty() || accumulated_cloud_->size() < 5000) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for more points to perform global search... current size: %zu", accumulated_cloud_->size());
      return;
    }
    std::swap(accumulated_cloud_, cloud_to_process);
  }

  // --- Phase 1: Coarse Search ---
  auto coarse_source = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *cloud_to_process, global_search_coarse_leaf_size_);

  small_gicp::estimate_covariances_omp(*coarse_source, num_neighbors_, num_threads_);

  if (!coarse_source || coarse_source->empty()) {
    return;
  }

  std::vector<Eigen::Isometry3d> coarse_candidates;
  
  double search_x_min = map_filter_x_min_;
  double search_x_max = map_filter_x_max_;
  double search_y_min = map_filter_y_min_;
  double search_y_max = map_filter_y_max_;
  int search_threads = omp_get_max_threads(); // Full CPU for global search

  if (!enable_global_search_) {
    // 1-meter local search around last known pose (previous_result_t_)
    double origin_x = previous_result_t_.translation().x();
    double origin_y = previous_result_t_.translation().y();
    search_x_min = origin_x - 2.0;
    search_x_max = origin_x + 2.0;
    search_y_min = origin_y - 2.0;
    search_y_max = origin_y + 2.0;
    search_threads = num_threads_; // Limit CPU for local search
    RCLCPP_INFO(this->get_logger(), "Global search disabled. Performing 1-meter local initialization around [%.2f, %.2f]...", origin_x, origin_y);
  }

  double step = enable_global_search_ ? global_search_coarse_step_ : global_search_step_;
  int samples_x = std::max(1, static_cast<int>(std::round((search_x_max - search_x_min) / step)) + 1);
  int samples_y = std::max(1, static_cast<int>(std::round((search_y_max - search_y_min) / step)) + 1);
  
  double x_step = (samples_x > 1) ? ((search_x_max - search_x_min) / (samples_x - 1)) : 0.0;
  double y_step = (samples_y > 1) ? ((search_y_max - search_y_min) / (samples_y - 1)) : 0.0;
  
  int yaw_samples = enable_global_search_ ? global_search_coarse_yaw_samples_ : global_search_yaw_samples_;
  double yaw_step = 2.0 * M_PI / std::max(1, yaw_samples);

  RCLCPP_INFO(this->get_logger(), "Global search grid: %dx%d samples (step: %.2fm), %d yaw samples", samples_x, samples_y, step, yaw_samples);

  for (int ix = 0; ix < samples_x; ++ix) {
    double x = search_x_min + ix * x_step;
    for (int iy = 0; iy < samples_y; ++iy) {
      double y = search_y_min + iy * y_step;
      for (int iyaw = 0; iyaw < yaw_samples; ++iyaw) {
        double yaw = iyaw * yaw_step;
        Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
        guess.translation() << x, y, previous_result_t_.translation().z();
        guess.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        coarse_candidates.push_back(guess);
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Starting coarse search with %zu candidates...", coarse_candidates.size());
  auto start_time = std::chrono::high_resolution_clock::now();

  std::mutex top_mutex;
  struct CandidateResult {
      double score;
      Eigen::Isometry3d pose;
      bool operator<(const CandidateResult& other) const {
          return score < other.score;
      }
  };
  std::vector<CandidateResult> top_candidates;

  #pragma omp parallel for num_threads(search_threads)
  for (size_t i = 0; i < coarse_candidates.size(); ++i) {
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> local_reg;
    local_reg.reduction.num_threads = 1; 
    local_reg.rejector.max_dist_sq = max_dist_sq_;
    local_reg.optimizer.max_iterations = global_search_coarse_iters_;

    auto result = local_reg.align(*target_, *coarse_source, *target_tree_, coarse_candidates[i]);
    
    if (result.num_inliers > 0) {
      size_t total_points = coarse_source->size();
      double score = (result.error + (total_points - result.num_inliers) * max_dist_sq_) / total_points;
      
      std::lock_guard<std::mutex> lock(top_mutex);
      top_candidates.push_back({score, result.T_target_source});
    }
  }

  // 移出多线程循环外部进行排序，彻底消除多线程锁等待和排序的性能瓶颈
  std::sort(top_candidates.begin(), top_candidates.end());
  if (top_candidates.size() > 5) {
      top_candidates.resize(5);
  }

  if (top_candidates.empty()) {
      RCLCPP_WARN(this->get_logger(), "Coarse search failed to find any valid candidates! Retrying next frame...");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Coarse search finished. Best score: %f. Starting fine search...", top_candidates.front().score);

  // --- Phase 2: Fine Search ---
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *cloud_to_process, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  double best_score = std::numeric_limits<double>::max();
  Eigen::Isometry3d best_pose = Eigen::Isometry3d::Identity();
  bool found_valid = false;

  #pragma omp parallel for num_threads(search_threads)
  for (size_t i = 0; i < top_candidates.size(); ++i) {
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> local_reg;
    local_reg.reduction.num_threads = 1; 
    local_reg.rejector.max_dist_sq = max_dist_sq_;
    local_reg.optimizer.max_iterations = global_search_fine_iters_;

    auto result = local_reg.align(*target_, *source_, *target_tree_, top_candidates[i].pose);
    
    if (result.num_inliers > 0) {
      size_t total_points = source_->size();
      double score = (result.error + (total_points - result.num_inliers) * max_dist_sq_) / total_points;
      
      std::lock_guard<std::mutex> lock(top_mutex);
      if (score < best_score) {
        best_score = score;
        best_pose = result.T_target_source;
        found_valid = true;
      }
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  if (found_valid) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    previous_result_t_ = result_t_ = best_pose;
    global_search_done_ = true;
    RCLCPP_INFO(this->get_logger(), "==========================================================");
    RCLCPP_INFO(this->get_logger(), "★ ★ ★ GLOBAL INITIALIZATION SUCCESSFUL ★ ★ ★");
    RCLCPP_INFO(this->get_logger(), "Best score: %f. Time taken: %ld ms", best_score, duration);
    RCLCPP_INFO(this->get_logger(), "==========================================================");
  } else {
    RCLCPP_WARN(this->get_logger(), "Fine search failed to find a valid pose! Retrying next frame...");
  }

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    accumulated_cloud_->clear();
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
