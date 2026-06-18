#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>

class EkfMapMonitor : public rclcpp::Node {
public:
    EkfMapMonitor() : Node("ekf_map_monitor") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 定时器：每 1000ms (1秒) 执行一次打印
        print_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() { this->print_callback(); });

        RCLCPP_INFO(this->get_logger(), "EKF 全局坐标监控器已启动...");
    }

private:
    void print_callback() {
        bool is_gicp_ready = false;
        try {
            // 通过检查 GICP 是否发出了 TF 来判断全局重定位是否成功初始化
            tf_buffer_->lookupTransform("map_gicp", "odom_gicp", tf2::TimePointZero);
            is_gicp_ready = true;
        } catch (const tf2::TransformException & ex) {
            is_gicp_ready = false;
        }

        try {
            // 查询最终经过 EKF 滤波的 map 到 base_link 的绝对坐标
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            double x = transform.transform.translation.x;
            double y = transform.transform.translation.y;
            double z = transform.transform.translation.z;

            if (is_gicp_ready) {
                // 打印绿色，表示已获得绝对定位
                RCLCPP_INFO(this->get_logger(), "\033[1;32m[GICP已初始化 - 狗的绝对坐标]\033[0m X: %8.3f m | Y: %8.3f m | Z: %8.3f m", x, y, z);
            } else {
                // 打印黄色，表示还在靠里程计盲推
                RCLCPP_INFO(this->get_logger(), "\033[1;33m[初始化中 - 仅靠轮推临时坐标]\033[0m X: %8.3f m | Y: %8.3f m | Z: %8.3f m", x, y, z);
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "等待 EKF 发出 map 到 base_link 的变换树...");
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr print_timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfMapMonitor>());
    rclcpp::shutdown();
    return 0;
}
