/**
 * @file decision_comm.cpp
 * @brief 决策通信模块 - 云边端协同通信核心
 * 
 * 功能概述：
 * 1. 设计决策指令的数据格式
 * 2. 实现可靠的UDP通信机制
 * 3. 处理网络延迟和丢包
 * 4. 支持双向通信（云端->边缘端）
 * 
 * 通信协议设计：
 * - 使用UDP协议减少延迟
 * - 添加序号、校验和、时间戳
 * - 实现自动重传机制
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <chrono>
#include <mutex>
#include <atomic>
#include <thread>
#include <queue>
#include <set>

using namespace std::chrono_literals;

/**
 * @brief 决策指令数据结构（二进制格式）
 */
#pragma pack(push, 1)
struct DecisionPacket {
    uint8_t header[2];          // 包头标识 0xAA 0x55
    uint8_t command;            // 命令类型
    uint8_t priority;           // 优先级 (0-255)
    float param1;               // 参数1 (线速度)
    float param2;               // 参数2 (角速度)
    uint32_t sequence;          // 序列号
    uint32_t timestamp;         // 时间戳 (ms)
    uint16_t checksum;          // 校验和
    uint8_t footer;             // 包尾标识 0x55
};
#pragma pack(pop)

/**
 * @brief 确认包结构
 */
#pragma pack(push, 1)
struct AckPacket {
    uint8_t header[2];          // 0xBB 0x66
    uint32_t ack_sequence;      // 确认的序列号
    uint32_t timestamp;         // 时间戳
    uint16_t checksum;          // 校验和
};
#pragma pack(pop)

/**
 * @brief 命令类型定义
 */
enum CommandType {
    CMD_NONE = 0,
    CMD_CONTINUE = 1,
    CMD_STOP = 2,
    CMD_SLOW_DOWN = 3,
    CMD_TURN_LEFT = 4,
    CMD_TURN_RIGHT = 5,
    CMD_PEDESTRIAN_AVOID = 6,
    CMD_TRAFFIC_LIGHT_WAIT = 7
};

/**
 * @brief 通信参数
 */
struct CommParams {
    std::string remote_ip = "192.168.1.100";  // 边缘端IP
    int remote_port = 8889;
    int local_port = 8890;
    int timeout_ms = 500;
    int max_retries = 3;
    int ack_timeout_ms = 100;
};

/**
 * @brief 决策通信节点类
 */
class DecisionCommNode : public rclcpp::Node {
public:
    DecisionCommNode() : Node("decision_comm_node"), running_(true) {
        // 声明参数
        this->declare_parameter("remote_ip", params_.remote_ip);
        this->declare_parameter("remote_port", params_.remote_port);
        this->declare_parameter("local_port", params_.local_port);
        
        params_.remote_ip = this->get_parameter("remote_ip").as_string();
        params_.remote_port = this->get_parameter("remote_port").as_int();
        params_.local_port = this->get_parameter("local_port").as_int();
        
        // 初始化UDP Socket
        initSocket();
        
        // 创建订阅者（接收云端决策）
        decision_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/decision_command", 10,
            std::bind(&DecisionCommNode::decisionCallback, this, std::placeholders::_1));
        
        // 创建发布者（发布到边缘端执行）
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_decision", 10);
        
        // 创建发送线程
        send_thread_ = std::thread(&DecisionCommNode::sendLoop, this);
        
        // 创建接收线程
        recv_thread_ = std::thread(&DecisionCommNode::recvLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "Decision comm node initialized");
        RCLCPP_INFO(this->get_logger(), "Remote: %s:%d", 
                   params_.remote_ip.c_str(), params_.remote_port);
    }
    
    ~DecisionCommNode() {
        running_ = false;
        
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:
    /**
     * @brief 初始化UDP Socket
     */
    void initSocket() {
        // 创建UDP Socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }
        
        // 设置非阻塞模式
        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
        
        // 绑定本地端口
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(params_.local_port);
        
        if (bind(sock_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind port %d", params_.local_port);
            close(sock_);
            sock_ = -1;
            return;
        }
        
        // 设置远程地址
        memset(&remote_addr_, 0, sizeof(remote_addr_));
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(params_.remote_port);
        inet_pton(AF_INET, params_.remote_ip.c_str(), &remote_addr_.sin_addr);
        
        RCLCPP_INFO(this->get_logger(), "Socket initialized on port %d", params_.local_port);
    }
    
    /**
     * @brief 决策回调（从云端接收）
     */
    void decisionCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        // 创建决策包
        DecisionPacket packet;
        memset(&packet, 0, sizeof(packet));
        
        packet.header[0] = 0xAA;
        packet.header[1] = 0x55;
        packet.command = static_cast<uint8_t>(msg->data);
        packet.priority = 100;  // 云端决策优先级较高
        packet.param1 = 0.0f;
        packet.param2 = 0.0f;
        packet.sequence = sequence_++;
        packet.timestamp = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
        
        // 根据命令类型设置参数
        switch (msg->data) {
            case CMD_STOP:
                packet.param1 = 0.0f;  // 线速度
                break;
            case CMD_SLOW_DOWN:
                packet.param1 = 0.2f;  // 降低到0.2m/s
                break;
            case CMD_CONTINUE:
                packet.param1 = 0.5f;  // 正常速度
                break;
            default:
                break;
        }
        
        // 计算校验和
        packet.checksum = calculateChecksum(
            reinterpret_cast<uint8_t*>(&packet), 
            sizeof(packet) - sizeof(packet.checksum) - sizeof(packet.footer));
        packet.footer = 0x55;
        
        // 加入发送队列
        {
            std::lock_guard<std::mutex> lock(send_queue_mutex_);
            send_queue_.push(packet);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Queued command %d, seq %u", 
                    packet.command, packet.sequence);
    }
    
    /**
     * @brief 发送线程主循环
     */
    void sendLoop() {
        while (running_) {
            DecisionPacket packet;
            bool has_packet = false;
            
            // 从队列获取数据包
            {
                std::lock_guard<std::mutex> lock(send_queue_mutex_);
                if (!send_queue_.empty()) {
                    packet = send_queue_.front();
                    send_queue_.pop();
                    has_packet = true;
                }
            }
            
            if (has_packet) {
                // 发送数据包（带重传）
                sendWithRetry(packet);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    /**
     * @brief 带重传的发送
     */
    void sendWithRetry(const DecisionPacket& packet) {
        for (int retry = 0; retry < params_.max_retries; retry++) {
            // 发送数据包
            ssize_t sent = sendto(sock_, &packet, sizeof(packet), 0,
                                  (struct sockaddr*)&remote_addr_, 
                                  sizeof(remote_addr_));
            
            if (sent < 0) {
                RCLCPP_WARN(this->get_logger(), "Send failed, retry %d", retry);
                std::this_thread::sleep_for(std::chrono::milliseconds(params_.ack_timeout_ms));
                continue;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Sent packet seq %u, command %d",
                        packet.sequence, packet.command);
            
            // 等待确认（非阻塞检查）
            bool acked = false;
            auto start = std::chrono::steady_clock::now();
            
            while (!acked) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start).count();
                
                if (elapsed > params_.ack_timeout_ms) {
                    break;  // 超时
                }
                
                // 检查是否收到确认
                {
                    std::lock_guard<std::mutex> lock(ack_mutex_);
                    if (ack_set_.count(packet.sequence) > 0) {
                        acked = true;
                        ack_set_.erase(packet.sequence);
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            
            if (acked) {
                RCLCPP_DEBUG(this->get_logger(), "Packet %u acknowledged", packet.sequence);
                return;
            }
            
            RCLCPP_WARN(this->get_logger(), "Packet %u no ACK, retry %d", 
                       packet.sequence, retry);
        }
        
        RCLCPP_ERROR(this->get_logger(), "Packet %u send failed after %d retries",
                    packet.sequence, params_.max_retries);
    }
    
    /**
     * @brief 接收线程主循环
     */
    void recvLoop() {
        uint8_t buffer[1024];
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        
        while (running_) {
            // 接收数据
            ssize_t received = recvfrom(sock_, buffer, sizeof(buffer), 0,
                                        (struct sockaddr*)&from_addr, &from_len);
            
            if (received > 0) {
                // 解析数据包
                if (received == sizeof(AckPacket)) {
                    // 处理确认包
                    AckPacket* ack = reinterpret_cast<AckPacket*>(buffer);
                    if (ack->header[0] == 0xBB && ack->header[1] == 0x66) {
                        uint16_t expected_crc = calculateChecksum(
                            reinterpret_cast<uint8_t*>(ack),
                            sizeof(AckPacket) - sizeof(ack->checksum));
                        
                        if (ack->checksum == expected_crc) {
                            std::lock_guard<std::mutex> lock(ack_mutex_);
                            ack_set_.insert(ack->ack_sequence);
                            RCLCPP_DEBUG(this->get_logger(), 
                                        "Received ACK for seq %u", ack->ack_sequence);
                        }
                    }
                } else if (received == sizeof(DecisionPacket)) {
                    // 处理决策包（边缘端发来的）
                    DecisionPacket* pkt = reinterpret_cast<DecisionPacket*>(buffer);
                    if (pkt->header[0] == 0xAA && pkt->header[1] == 0x55) {
                        handleDecisionPacket(pkt);
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    /**
     * @brief 处理接收到的决策包
     */
    void handleDecisionPacket(DecisionPacket* pkt) {
        // 验证校验和
        uint16_t expected_crc = calculateChecksum(
            reinterpret_cast<uint8_t*>(pkt),
            sizeof(DecisionPacket) - sizeof(pkt->checksum) - sizeof(pkt->footer));
        
        if (pkt->checksum != expected_crc) {
            RCLCPP_WARN(this->get_logger(), "Invalid checksum, dropping packet");
            return;
        }
        
        // 发送确认
        sendAck(pkt->sequence);
        
        // 发布速度指令
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = pkt->param1;
        cmd.angular.z = pkt->param2;
        cmd_vel_pub_->publish(cmd);
        
        RCLCPP_INFO(this->get_logger(), "Executed command %d: linear=%.2f, angular=%.2f",
                   pkt->command, pkt->param1, pkt->param2);
    }
    
    /**
     * @brief 发送确认包
     */
    void sendAck(uint32_t sequence) {
        AckPacket ack;
        ack.header[0] = 0xBB;
        ack.header[1] = 0x66;
        ack.ack_sequence = sequence;
        ack.timestamp = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
        ack.checksum = calculateChecksum(
            reinterpret_cast<uint8_t*>(&ack),
            sizeof(ack) - sizeof(ack.checksum));
        
        sendto(sock_, &ack, sizeof(ack), 0,
               (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    }
    
    /**
     * @brief 计算校验和
     */
    uint16_t calculateChecksum(const uint8_t* data, size_t len) {
        uint16_t sum = 0;
        for (size_t i = 0; i < len; i++) {
            sum += data[i];
        }
        return sum;
    }
    
    // 成员变量
    CommParams params_;
    int sock_ = -1;
    struct sockaddr_in remote_addr_;
    std::atomic<bool> running_;
    std::atomic<uint32_t> sequence_{0};
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr decision_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    std::thread send_thread_;
    std::thread recv_thread_;
    
    std::mutex send_queue_mutex_;
    std::queue<DecisionPacket> send_queue_;
    
    std::mutex ack_mutex_;
    std::set<uint32_t> ack_set_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
