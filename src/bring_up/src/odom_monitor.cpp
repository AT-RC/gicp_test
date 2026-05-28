#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iomanip>
#include <mutex>

class OdomMonitor : public rclcpp::Node {
public:
    OdomMonitor() : Node("odom_monitor"), has_data_(false) {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "lidar_odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_pos_ = msg->pose.pose.position;
                has_data_ = true;
            });
        
        // 创建定时器：每 1000ms (1秒) 执行一次打印
        print_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() { this->print_callback(); });

        RCLCPP_INFO(this->get_logger(), "C++ 坐标监控器已启动，每秒打印一次位置数据...");
    }

private:
    void print_callback() {
        if (!has_data_) {
            RCLCPP_WARN(this->get_logger(), "等待里程计数据中...");
            return;
        }

        geometry_msgs::msg::Point pos;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            pos = latest_pos_;
        }

        RCLCPP_INFO(this->get_logger(), "[实时坐标] X: %8.3f m | Y: %8.3f m | Z: %8.3f m", 
                    pos.x, pos.y, pos.z);
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr print_timer_;
    
    std::mutex data_mutex_;
    geometry_msgs::msg::Point latest_pos_;
    bool has_data_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomMonitor>());
    rclcpp::shutdown();
    return 0;
}
