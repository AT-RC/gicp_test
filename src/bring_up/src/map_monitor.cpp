#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>

class MapMonitor : public rclcpp::Node {
public:
    MapMonitor() : Node("map_monitor") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 定时器：每 1000ms (1秒) 执行一次打印
        print_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() { this->print_callback(); });

        RCLCPP_INFO(this->get_logger(), "地图坐标监控器已启动，每秒打印一次 map -> base_link 的位置...");
    }

private:
    void print_callback() {
        geometry_msgs::msg::TransformStamped transform;
        try {
            // 查询 map 到 base_link 的 TF
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            
            double x = transform.transform.translation.x;
            double y = transform.transform.translation.y;
            double z = transform.transform.translation.z;

            RCLCPP_INFO(this->get_logger(), "[全局坐标] X: %8.3f m | Y: %8.3f m | Z: %8.3f m", 
                        x, y, z);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "等待 map 到 base_link 的 TF 坐标系转换...");
        }
    }
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr print_timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMonitor>());
    rclcpp::shutdown();
    return 0;
}
