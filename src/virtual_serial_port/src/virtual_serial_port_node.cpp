#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "virtual_serial_port/cdc_trans.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>

// 发送给下位机的位姿数据包结构 (20字节)
#pragma pack(push, 1)
struct PosePacket {
    uint8_t header[2]; // 帧头
    float x;           // x坐标 m
    float y;           // y坐标 m
    float z;           // z坐标 m
    float yaw;         // 偏航角 rad
    uint8_t checksum;  // 校验和 (x,y,z,yaw的字节累加)
    uint8_t tail;      // 帧尾
};
#pragma pack(pop)

class VirtualSerialPortNode : public rclcpp::Node
{
public:
    VirtualSerialPortNode() : Node("virtual_serial_port_node"), running_(true)
    {
        // ... (保持构造函数其余部分不变)
        this->declare_parameter<int>("usb_vid", 0x0483);  
        this->declare_parameter<int>("usb_pid", 0x5740);  
        this->declare_parameter<int>("send_interval_ms", 10); // 默认100Hz
        this->declare_parameter<std::string>("odom_frame", "camera_init");
        this->declare_parameter<std::string>("base_frame", "aft_mapped");
        
        // 获取参数
        usb_vid_ = static_cast<uint16_t>(this->get_parameter("usb_vid").as_int());
        usb_pid_ = static_cast<uint16_t>(this->get_parameter("usb_pid").as_int());
        send_interval_ms_ = this->get_parameter("send_interval_ms").as_int();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        
        // 初始化位姿数据
        current_pose_ = {0.0f, 0.0f, 0.0f, 0.0f};
        
        // 初始化TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 初始化USB-CDC传输
        cdc_trans_ = std::make_unique<CDCTrans>();
        
        if (cdc_trans_->open(usb_vid_, usb_pid_)) {
            RCLCPP_INFO(this->get_logger(), "USB-CDC设备打开成功 VID:0x%04X PID:0x%04X", usb_vid_, usb_pid_);
        } else {
            RCLCPP_WARN(this->get_logger(), "等待USB-CDC设备接入...");
        }
        
        // 启动处理线程
        send_thread_ = std::thread(&VirtualSerialPortNode::send_thread_func, this);
        usb_thread_ = std::thread(&VirtualSerialPortNode::usb_thread_func, this);
        tf_thread_ = std::thread(&VirtualSerialPortNode::tf_thread_func, this);
        
        RCLCPP_INFO(this->get_logger(), "串口发送节点已启动: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
    }
    
    ~VirtualSerialPortNode()
    {
        running_ = false;
        if (send_thread_.joinable()) send_thread_.join();
        if (usb_thread_.joinable()) usb_thread_.join();
        if (tf_thread_.joinable()) tf_thread_.join();
    }

private:
    void send_thread_func()
    {
        while (running_) {
            auto next_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(send_interval_ms_);
            
            PosePacket packet;
            packet.header[0] = 0x5A;
            packet.header[1] = 0xA5;
            packet.tail = 0xEE;
            
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                packet.x = current_pose_.x;
                packet.y = current_pose_.y;
                packet.z = current_pose_.z;
                packet.yaw = current_pose_.yaw;
            }
            
            // 计算校验和
            uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&packet.x);
            uint8_t sum = 0;
            for(size_t i=0; i < 4 * sizeof(float); i++) {
                sum += data_ptr[i];
            }
            packet.checksum = sum;
            
            // 执行发送
            cdc_trans_->send_struct(packet);
            
            std::this_thread::sleep_until(next_time);
        }
    }
    
    void usb_thread_func()
    {
        while (running_) {
            cdc_trans_->process_once();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void tf_thread_func()
    {
        while (running_) {
            try {
                // 查询最新变换
                geometry_msgs::msg::TransformStamped tf_stamped = 
                    tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
                
                float x = static_cast<float>(tf_stamped.transform.translation.x);
                float y = static_cast<float>(tf_stamped.transform.translation.y);
                float z = static_cast<float>(tf_stamped.transform.translation.z);
                
                auto& q = tf_stamped.transform.rotation;
                tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    current_pose_ = {x, y, z, static_cast<float>(yaw)};
                }
            } catch (tf2::TransformException &ex) {
                // 启动初期可能没有TF，静默等待
            }
            // 以200Hz频率查询位姿，确保发送时是最新的
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    
    struct PoseData {
        float x, y, z, yaw;
    };
    
    std::unique_ptr<CDCTrans> cdc_trans_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::thread send_thread_, usb_thread_, tf_thread_;
    std::atomic<bool> running_;
    std::mutex pose_mutex_;
    PoseData current_pose_;
    
    uint16_t usb_vid_, usb_pid_;
    int send_interval_ms_;
    std::string odom_frame_, base_frame_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSerialPortNode>());
    rclcpp::shutdown();
    return 0;
}
