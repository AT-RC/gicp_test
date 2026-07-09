#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_msgs/msg/cmd.hpp>

class Nav2CmdBridge : public rclcpp::Node {
public:
    Nav2CmdBridge() : Node("nav2_cmd_bridge") {
        // Subscribe to Nav2's cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Nav2CmdBridge::cmd_vel_callback, this, std::placeholders::_1));

        // Publish to robot's command topic
        robot_cmd_pub_ = this->create_publisher<robot_msgs::msg::Cmd>(
            "robot_move_cmd", 10);

        RCLCPP_INFO(this->get_logger(), "Nav2 cmd_vel -> robot_move_cmd Bridge Started.");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        robot_msgs::msg::Cmd out_cmd;
        out_cmd.mode = 2; // kWalkMode
        out_cmd.vx = msg->linear.x;
        out_cmd.vy = msg->linear.y;
        out_cmd.vz = 0.0;
        out_cmd.wheel_vel = msg->angular.z; // mapped to yaw angular velocity
        
        robot_cmd_pub_->publish(out_cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr robot_cmd_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2CmdBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
