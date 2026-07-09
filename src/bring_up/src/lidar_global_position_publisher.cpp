#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class LidarGlobalPositionPublisher : public rclcpp::Node
{
public:
  LidarGlobalPositionPublisher()
  : Node("lidar_global_position_publisher")
  {
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "mid360_imu");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "lidar_global_position");
    const double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 20.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(topic_name_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishPosition(); });

    RCLCPP_INFO(
      this->get_logger(), "Publishing %s position in %s on %s",
      lidar_frame_.c_str(), global_frame_.c_str(), topic_name_.c_str());
  }

private:
  void publishPosition()
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        global_frame_, lidar_frame_, tf2::TimePointZero);

      geometry_msgs::msg::PointStamped position;
      position.header.stamp = this->now();
      position.header.frame_id = global_frame_;
      position.point.x = transform.transform.translation.x;
      position.point.y = transform.transform.translation.y;
      position.point.z = transform.transform.translation.z;

      position_pub_->publish(position);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for TF %s -> %s: %s",
        global_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    }
  }

  std::string global_frame_;
  std::string lidar_frame_;
  std::string topic_name_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarGlobalPositionPublisher>());
  rclcpp::shutdown();
  return 0;
}