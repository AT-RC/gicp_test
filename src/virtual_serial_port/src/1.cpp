#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include "virtual_serial_port/cdc_trans.hpp"
#include <thread>
#include <mutex>
#include <atomic>

// 爬楼梯动作类型
enum class ClimbAction : uint8_t {
    NONE = 0,      // 无动作
    CLIMB = 1,     // 上楼梯
    DESCEND = 2    // 下楼梯
};

// 发送给下位机的速度数据包结构
#pragma pack(push, 1)
struct VelocityPacket {
    float vx;          // x方向线速度 m/s
    float vy;          // y方向线速度 m/s  
    float omega;       // 角速度 rad/s
    uint8_t climb_action;  // 爬楼梯动作类型 (0=无, 1=上, 2=下)
    float climb_height;    // 爬楼梯高度 m
};
#pragma pack(pop)

// 从下位机接收的状态数据包结构
#pragma pack(push, 1)
struct StatusPacket {
    int32_t climber_status;  // 爬楼梯状态 (0=空闲, 1=执行中, 2=成功, 3=失败)
};
#pragma pack(pop)

class VirtualSerialPortNode : public rclcpp::Node
{
public:
    VirtualSerialPortNode() : Node("virtual_serial_port_node"), running_(true)
    {
        // 声明参数
        this->declare_parameter<int>("usb_vid", 0x0483);  // STM32 默认VID
        this->declare_parameter<int>("usb_pid", 0x5740);  // CDC默认PID
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        this->declare_parameter<int>("send_interval_ms", 8);  // 发送周期
        
        // 获取参数
        usb_vid_ = static_cast<uint16_t>(this->get_parameter("usb_vid").as_int());
        usb_pid_ = static_cast<uint16_t>(this->get_parameter("usb_pid").as_int());
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        send_interval_ms_ = this->get_parameter("send_interval_ms").as_int();
        
        // 初始化速度和爬楼梯状态
        current_velocity_ = {0.0f, 0.0f, 0.0f};
        current_climb_action_ = ClimbAction::NONE;
        current_climb_height_ = 0.0f;
        
        // 初始化CDC设备
        cdc_trans_ = std::make_unique<CDCTrans>();
        
        // 尝试打开USB设备
        if (cdc_trans_->open(usb_vid_, usb_pid_)) {
            RCLCPP_INFO(this->get_logger(), "USB-CDC设备打开成功 VID:0x%04X PID:0x%04X", usb_vid_, usb_pid_);
        } else {
            RCLCPP_WARN(this->get_logger(), "USB-CDC设备打开失败，将持续尝试重连");
        }
        
        // 注册接收回调
        cdc_trans_->regeiser_recv_cb([this](const uint8_t* data, int size) {
            this->on_data_received(data, size);
        });
        
        // 订阅cmd_vel话题
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&VirtualSerialPortNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // 订阅爬楼梯话题
        climb_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/AT_R2/climb_stair", 10,
            std::bind(&VirtualSerialPortNode::climb_callback, this, std::placeholders::_1));
        
        // 订阅下楼梯话题
        descend_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/AT_R2/descend_stair", 10,
            std::bind(&VirtualSerialPortNode::descend_callback, this, std::placeholders::_1));
        
        // 创建爬楼梯状态发布器
        climber_status_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/AT_R2/climber_status", 10);
        
        // 启动发送线程（独立线程，8ms周期）
        send_thread_ = std::thread(&VirtualSerialPortNode::send_thread_func, this);
        
        // 启动USB事件处理线程
        usb_thread_ = std::thread(&VirtualSerialPortNode::usb_thread_func, this);
        
        RCLCPP_INFO(this->get_logger(), "虚拟串口节点已启动，订阅话题: %s, 发送周期: %dms", 
            cmd_vel_topic.c_str(), send_interval_ms_);
        RCLCPP_INFO(this->get_logger(), "已订阅爬楼梯话题: /AT_R2/climb_stair, /AT_R2/descend_stair");
        RCLCPP_INFO(this->get_logger(), "已创建状态发布器: /AT_R2/climber_status");
    }
    
    ~VirtualSerialPortNode()
    {
        running_ = false;
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
        if (usb_thread_.joinable()) {
            usb_thread_.join();
        }
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 只更新速度值，不在回调中发送
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_velocity_.vx = static_cast<float>(msg->linear.x);
        current_velocity_.vy = static_cast<float>(msg->linear.y);
        current_velocity_.omega = static_cast<float>(msg->angular.z);
    }
    
    void climb_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_climb_action_ = ClimbAction::CLIMB;
        current_climb_height_ = static_cast<float>(msg->data);
        
        RCLCPP_INFO(this->get_logger(), "收到爬楼梯指令: 高度 %.2f m", msg->data);
    }
    
    void descend_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_climb_action_ = ClimbAction::DESCEND;
        current_climb_height_ = static_cast<float>(msg->data);
        
        RCLCPP_INFO(this->get_logger(), "收到下楼梯指令: 高度 %.2f m", msg->data);
    }
    
    void send_thread_func()
    {
        using namespace std::chrono_literals;
        RCLCPP_INFO(this->get_logger(), "发送线程已启动");
        
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            
            // 构建数据包
            VelocityPacket packet;
            {
                std::lock_guard<std::mutex> lock(velocity_mutex_);
                packet.vx = current_velocity_.vx;
                packet.vy = current_velocity_.vy;
                packet.omega = current_velocity_.omega;
                packet.climb_action = static_cast<uint8_t>(current_climb_action_);
                packet.climb_height = current_climb_height_;
                
                // 发送后清除爬楼梯指令（单次触发）
                if (current_climb_action_ != ClimbAction::NONE) {
                    RCLCPP_DEBUG(this->get_logger(), 
                        "发送爬楼梯指令: action=%d, height=%.2f", 
                        static_cast<int>(current_climb_action_), current_climb_height_);
                    // 注意：不立即清除，等待下位机确认后再清除
                    // current_climb_action_ = ClimbAction::NONE;
                    // current_climb_height_ = 0.0f;
                }
            }
            
            // 发送并检查结果
            if (!cdc_trans_->send_struct(packet)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "发送速度指令失败");
            }
            
            std::this_thread::sleep_until(now + 8ms);
        }
        
        RCLCPP_INFO(this->get_logger(), "发送线程已退出");
    }
    
    void usb_thread_func()
    {
        RCLCPP_INFO(this->get_logger(), "USB事件处理线程已启动");
        
        while (running_) {
            cdc_trans_->process_once();
        }
        
        RCLCPP_INFO(this->get_logger(), "USB事件处理线程已退出");
    }
    
    void on_data_received(const uint8_t* data, int size)
    {
        RCLCPP_DEBUG(this->get_logger(), "收到下位机数据，长度: %d", size);
        
        // 检查数据包大小是否匹配
        if (size == sizeof(StatusPacket)) {
            StatusPacket status;
            std::memcpy(&status, data, sizeof(StatusPacket));
            
            // 发布爬楼梯状态
            auto msg = std_msgs::msg::Int32();
            msg.data = status.climber_status;
            climber_status_pub_->publish(msg);
            
            // 打印状态变化
            static int32_t last_status = -1;
            if (status.climber_status != last_status) {
                const char* status_names[] = {"IDLE", "RUNNING", "SUCCESS", "FAILED"};
                if (status.climber_status >= 0 && status.climber_status <= 3) {
                    RCLCPP_INFO(this->get_logger(), 
                        "爬楼梯状态更新: %s (%d)", 
                        status_names[status.climber_status], 
                        status.climber_status);
                }
                last_status = status.climber_status;
                
                // 如果任务完成（成功或失败），清除爬楼梯指令
                if (status.climber_status == 2 || status.climber_status == 3) {
                    std::lock_guard<std::mutex> lock(velocity_mutex_);
                    current_climb_action_ = ClimbAction::NONE;
                    current_climb_height_ = 0.0f;
                    RCLCPP_DEBUG(this->get_logger(), "已清除爬楼梯指令");
                }
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "收到的数据包大小不匹配: 期望 %zu 字节，实际 %d 字节", 
                sizeof(StatusPacket), size);
        }
    }
    
    // 当前速度结构
    struct Velocity {
        float vx;
        float vy;
        float omega;
    };
    
    std::unique_ptr<CDCTrans> cdc_trans_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr climb_sub_;
    rclcpp::Subscriptiocccn<std_msgs::msg::Float64>::SharedPtr descend_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr climber_status_pub_;
    
    std::thread send_thread_;
    std::thread usb_thread_;
    std::atomic<bool> running_;
    
    std::mutex velocity_mutex_;
    Velocity current_velocity_;
    ClimbAction current_climb_action_;
    float current_climb_height_;
    
    uint16_t usb_vid_;
    uint16_t usb_pid_;
    int send_interval_ms_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSerialPortNode>());
    rclcpp::shutdown();
    return 0;
}