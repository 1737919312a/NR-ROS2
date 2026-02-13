/**
 * @file microros_bridge_node.cpp
 * @brief Micro-ROS Agent 桥接节点 - 边缘端核心通信模块
 * 
 * 功能概述：
 * 1. 运行 Micro-ROS Agent，汇聚 MCU 数据
 * 2. 通过 USB 串口连接主机 MCU (ESP32-S3)
 * 3. 通过 WiFi UDP 连接从机 MCU (ESP32-S3)
 * 4. 发布传感器数据到 ROS 2 话题
 * 5. 转发 ROS 2 指令到 MCU
 * 
 * 通信架构：
 * 
 *   [主机MCU] <--USB Serial--> [Micro-ROS] <--ROS2--> [节点]
 *                                            Agent
 *   [从机MCU] <--WiFi UDP----> [Bridge]
 * 
 * 硬件配置：
 * - USB串口：/dev/ttyUSB0 或 /dev/ttyACM0
 * - 波特率：921600 (高速模式)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <memory>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

/* ==================== 常量定义 ==================== */

// 串口配置
#define SERIAL_DEVICE_PRIMARY   "/dev/ttyUSB0"
#define SERIAL_DEVICE_FALLBACK  "/dev/ttyACM0"
#define SERIAL_BAUD_RATE        921600
#define SERIAL_BUFFER_SIZE      4096

// UDP配置
#define UDP_PORT_SLAVE          8889
#define UDP_BUFFER_SIZE         2048
#define UDP_TIMEOUT_MS          1000

// 数据超时
#define SENSOR_TIMEOUT_MS       1000

/* ==================== 数据结构定义 ==================== */

/**
 * @brief MCU状态信息
 */
struct MCUStatus {
    uint8_t id = 0;
    bool connected = false;
    uint32_t last_heartbeat = 0;
    float battery_voltage = 12.0f;
    uint8_t error_code = 0;
    
    struct {
        float x = 0, y = 0, theta = 0;
        float v_linear = 0, v_angular = 0;
    } odometry;
    
    struct {
        float min_distance = 5.0f;
        bool emergency_stop = false;
    } obstacle;
};

/**
 * @brief 桥接参数配置
 */
struct BridgeParams {
    std::string serial_device = SERIAL_DEVICE_PRIMARY;
    int serial_baud = SERIAL_BAUD_RATE;
    int udp_port_slave = UDP_PORT_SLAVE;
    std::string cmd_vel_topic = "/cmd_vel";
    std::string scan_topic = "/scan";
    std::string odom_topic = "/odom";
    std::string obstacle_topic = "/obstacle_status";
};

/* ==================== Micro-ROS Agent 桥接节点 ==================== */

class MicroROSBridgeNode : public rclcpp::Node {
public:
    MicroROSBridgeNode() : Node("microros_bridge_node"), 
                           running_(true),
                           serial_fd_(-1),
                           udp_sock_(-1) {
        declareParameters();
        initPublishers();
        initSubscribers();
        
        if (!initSerial()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
        }
        
        if (!initUDP()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
        }
        
        // 启动通信线程
        serial_thread_ = std::thread(&MicroROSBridgeNode::serialReceiveLoop, this);
        udp_thread_ = std::thread(&MicroROSBridgeNode::udpReceiveLoop, this);
        monitor_thread_ = std::thread(&MicroROSBridgeNode::monitorLoop, this);
        
        // 定时器
        heartbeat_timer_ = this->create_wall_timer(
            100ms, std::bind(&MicroROSBridgeNode::heartbeatCallback, this));
        
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&MicroROSBridgeNode::statusCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Micro-ROS Bridge Node ===");
        RCLCPP_INFO(this->get_logger(), "Serial: %s @ %d baud", 
                   params_.serial_device.c_str(), params_.serial_baud);
        RCLCPP_INFO(this->get_logger(), "UDP port: %d", params_.udp_port_slave);
    }
    
    ~MicroROSBridgeNode() {
        running_ = false;
        
        if (serial_thread_.joinable()) serial_thread_.join();
        if (udp_thread_.joinable()) udp_thread_.join();
        if (monitor_thread_.joinable()) monitor_thread_.join();
        
        if (serial_fd_ >= 0) close(serial_fd_);
        if (udp_sock_ >= 0) close(udp_sock_);
        
        RCLCPP_INFO(this->get_logger(), "Bridge node shutdown");
    }

private:
    void declareParameters() {
        this->declare_parameter("serial_device", params_.serial_device);
        this->declare_parameter("serial_baud", params_.serial_baud);
        this->declare_parameter("udp_port_slave", params_.udp_port_slave);
        
        params_.serial_device = this->get_parameter("serial_device").as_string();
        params_.serial_baud = this->get_parameter("serial_baud").as_int();
        params_.udp_port_slave = this->get_parameter("udp_port_slave").as_int();
    }
    
    void initPublishers() {
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            params_.scan_topic, rclcpp::QoS(10));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            params_.odom_topic, rclcpp::QoS(10));
        obstacle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            params_.obstacle_topic, rclcpp::QoS(10));
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/bridge_status", rclcpp::QoS(10));
        connection_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/bridge_connected", rclcpp::QoS(10));
    }
    
    void initSubscribers() {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            params_.cmd_vel_topic, rclcpp::QoS(10),
            std::bind(&MicroROSBridgeNode::cmdVelCallback, this, std::placeholders::_1));
        
        follow_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/follow_cmd", rclcpp::QoS(10),
            std::bind(&MicroROSBridgeNode::followCmdCallback, this, std::placeholders::_1));
    }
    
    bool initSerial() {
        serial_fd_ = open(params_.serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        
        if (serial_fd_ < 0) {
            RCLCPP_WARN(this->get_logger(), "Cannot open %s, trying fallback", 
                       params_.serial_device.c_str());
            serial_fd_ = open(SERIAL_DEVICE_FALLBACK, O_RDWR | O_NOCTTY | O_NONBLOCK);
            
            if (serial_fd_ < 0) {
                RCLCPP_ERROR(this->get_logger(), "Cannot open serial: %s", strerror(errno));
                return false;
            }
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed");
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        
        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);
        
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed");
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        
        tcflush(serial_fd_, TCIOFLUSH);
        RCLCPP_INFO(this->get_logger(), "Serial initialized: fd=%d", serial_fd_);
        return true;
    }
    
    bool initUDP() {
        udp_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP socket failed: %s", strerror(errno));
            return false;
        }
        
        int flags = fcntl(udp_sock_, F_GETFL, 0);
        fcntl(udp_sock_, F_SETFL, flags | O_NONBLOCK);
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(params_.udp_port_slave);
        
        if (bind(udp_sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP bind failed: %s", strerror(errno));
            close(udp_sock_);
            udp_sock_ = -1;
            return false;
        }
        
        memset(&slave_addr_, 0, sizeof(slave_addr_));
        slave_addr_len_ = sizeof(slave_addr_);
        
        RCLCPP_INFO(this->get_logger(), "UDP initialized on port %d", params_.udp_port_slave);
        return true;
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (serial_fd_ < 0) return;
        
        #pragma pack(push, 1)
        struct {
            uint8_t header[2] = {0xAA, 0x55};
            uint8_t type = 0x01;
            float linear_x;
            float linear_y;
            float angular_z;
            uint16_t checksum = 0;
        } pkt;
        #pragma pack(pop)
        
        pkt.linear_x = static_cast<float>(msg->linear.x);
        pkt.linear_y = static_cast<float>(msg->linear.y);
        pkt.angular_z = static_cast<float>(msg->angular.z);
        
        write(serial_fd_, &pkt, sizeof(pkt));
        RCLCPP_DEBUG(this->get_logger(), "Forwarded cmd_vel: %.2f, %.2f",
                    msg->linear.x, msg->angular.z);
    }
    
    void followCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (udp_sock_ < 0 || !slave_connected_) return;
        
        #pragma pack(push, 1)
        struct {
            uint8_t header[2] = {0xAA, 0x56};
            uint8_t type = 0x02;
            float linear_x;
            float angular_z;
            uint32_t timestamp;
            uint16_t checksum = 0;
        } pkt;
        #pragma pack(pop)
        
        pkt.linear_x = static_cast<float>(msg->linear.x);
        pkt.angular_z = static_cast<float>(msg->angular.z);
        pkt.timestamp = getTimestampMs();
        
        sendto(udp_sock_, &pkt, sizeof(pkt), 0,
               (struct sockaddr*)&slave_addr_, sizeof(slave_addr_));
        
        RCLCPP_DEBUG(this->get_logger(), "Forwarded follow_cmd: %.2f, %.2f",
                    msg->linear.x, msg->angular.z);
    }
    
    void serialReceiveLoop() {
        uint8_t buffer[SERIAL_BUFFER_SIZE];
        uint8_t packet_buf[256];
        size_t pkt_idx = 0;
        
        while (running_) {
            if (serial_fd_ < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            ssize_t n = read(serial_fd_, buffer, sizeof(buffer));
            
            if (n > 0) {
                host_status_.connected = true;
                host_status_.last_heartbeat = getTimestampMs();
                
                for (ssize_t i = 0; i < n; i++) {
                    uint8_t byte = buffer[i];
                    
                    if (pkt_idx == 0 && byte != 0xAA) continue;
                    if (pkt_idx == 1 && byte != 0x55) { pkt_idx = 0; continue; }
                    
                    packet_buf[pkt_idx++] = byte;
                    
                    if (pkt_idx >= 3) {
                        size_t expected = getPacketLength(packet_buf[2]);
                        if (pkt_idx >= expected) {
                            processSerialPacket(packet_buf, expected);
                            pkt_idx = 0;
                        }
                    }
                    
                    if (pkt_idx >= sizeof(packet_buf)) pkt_idx = 0;
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void udpReceiveLoop() {
        uint8_t buffer[UDP_BUFFER_SIZE];
        
        while (running_) {
            if (udp_sock_ < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            socklen_t addr_len = sizeof(slave_addr_);
            ssize_t n = recvfrom(udp_sock_, buffer, sizeof(buffer), 0,
                                (struct sockaddr*)&slave_addr_, &addr_len);
            
            if (n > 0) {
                slave_connected_ = true;
                slave_status_.connected = true;
                slave_status_.last_heartbeat = getTimestampMs();
                
                if (n >= 3 && buffer[0] == 0xAA && buffer[1] == 0x55) {
                    processUDPPacket(buffer, n);
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void monitorLoop() {
        while (running_) {
            uint32_t now = getTimestampMs();
            
            if (host_status_.connected && now - host_status_.last_heartbeat > SENSOR_TIMEOUT_MS) {
                host_status_.connected = false;
                RCLCPP_WARN(this->get_logger(), "Host MCU timeout");
            }
            
            if (slave_connected_ && now - slave_status_.last_heartbeat > SENSOR_TIMEOUT_MS) {
                slave_connected_ = false;
                slave_status_.connected = false;
                RCLCPP_WARN(this->get_logger(), "Slave MCU timeout");
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void processSerialPacket(const uint8_t* data, size_t len) {
        if (len < 3) return;
        
        switch (data[2]) {
            case 0x10: processLaserScan(data, len); break;
            case 0x11: processOdometry(data, len); break;
            case 0x12: processStatus(data, len, true); break;
            default: break;
        }
    }
    
    void processUDPPacket(const uint8_t* data, size_t len) {
        if (len < 3) return;
        
        switch (data[2]) {
            case 0x20: processObstacle(data, len); break;
            case 0x21: processStatus(data, len, false); break;
            default: break;
        }
    }
    
    void processLaserScan(const uint8_t* data, size_t len) {
        #pragma pack(push, 1)
        struct ScanPkt {
            uint8_t hdr[2]; uint8_t type;
            uint32_t ts;
            uint16_t count;
            float angle_min, angle_max, angle_inc;
            float range_min, range_max;
        };
        #pragma pack(pop)
        
        if (len < sizeof(ScanPkt)) return;
        
        auto* pkt = reinterpret_cast<const ScanPkt*>(data);
        
        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "laser_frame";
        msg.angle_min = pkt->angle_min;
        msg.angle_max = pkt->angle_max;
        msg.angle_increment = pkt->angle_inc;
        msg.scan_time = 1.0f / 12.0f;
        msg.range_min = pkt->range_min;
        msg.range_max = pkt->range_max;
        
        size_t offset = sizeof(ScanPkt);
        size_t count = (len - offset) / sizeof(float);
        msg.ranges.resize(count);
        
        const float* ranges = reinterpret_cast<const float*>(data + offset);
        for (size_t i = 0; i < count; i++) {
            msg.ranges[i] = ranges[i];
        }
        
        scan_pub_->publish(msg);
    }
    
    void processOdometry(const uint8_t* data, size_t len) {
        #pragma pack(push, 1)
        struct OdomPkt {
            uint8_t hdr[2]; uint8_t type;
            uint32_t ts;
            float x, y, theta;
            float v_lin, v_ang;
        };
        #pragma pack(pop)
        
        if (len < sizeof(OdomPkt)) return;
        
        auto* pkt = reinterpret_cast<const OdomPkt*>(data);
        
        host_status_.odometry.x = pkt->x;
        host_status_.odometry.y = pkt->y;
        host_status_.odometry.theta = pkt->theta;
        
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        
        msg.pose.pose.position.x = pkt->x;
        msg.pose.pose.position.y = pkt->y;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, pkt->theta);
        msg.pose.pose.orientation = tf2::toMsg(q);
        
        msg.twist.twist.linear.x = pkt->v_lin;
        msg.twist.twist.angular.z = pkt->v_ang;
        
        odom_pub_->publish(msg);
    }
    
    void processObstacle(const uint8_t* data, size_t len) {
        #pragma pack(push, 1)
        struct ObsPkt {
            uint8_t hdr[2]; uint8_t type;
            uint32_t ts;
            float min_dist;
            uint8_t emergency;
        };
        #pragma pack(pop)
        
        if (len < sizeof(ObsPkt)) return;
        
        auto* pkt = reinterpret_cast<const ObsPkt*>(data);
        
        slave_status_.obstacle.min_distance = pkt->min_dist;
        slave_status_.obstacle.emergency_stop = pkt->emergency;
        
        std_msgs::msg::Float32 msg;
        msg.data = pkt->min_dist;
        obstacle_pub_->publish(msg);
        
        if (pkt->emergency) {
            RCLCPP_WARN(this->get_logger(), "Slave emergency! dist=%.2f", pkt->min_dist);
        }
    }
    
    void processStatus(const uint8_t* data, size_t len, bool is_host) {
        #pragma pack(push, 1)
        struct StatPkt {
            uint8_t hdr[2]; uint8_t type;
            uint32_t ts;
            float bat_v;
            uint8_t err;
        };
        #pragma pack(pop)
        
        if (len < sizeof(StatPkt)) return;
        
        auto* pkt = reinterpret_cast<const StatPkt*>(data);
        
        MCUStatus& s = is_host ? host_status_ : slave_status_;
        s.battery_voltage = pkt->bat_v;
        s.error_code = pkt->err;
        s.last_heartbeat = getTimestampMs();
        s.connected = true;
        
        if (pkt->bat_v < 10.5f) {
            RCLCPP_WARN(this->get_logger(), "%s battery low: %.1fV",
                       is_host ? "Host" : "Slave", pkt->bat_v);
        }
    }
    
    void heartbeatCallback() {
        if (serial_fd_ >= 0) {
            uint8_t hb[] = {0xBB, 0x66, 0x00};
            write(serial_fd_, hb, sizeof(hb));
        }
        
        if (udp_sock_ >= 0 && slave_connected_) {
            uint8_t hb[] = {0xBB, 0x66, 0x00};
            sendto(udp_sock_, hb, sizeof(hb), 0,
                   (struct sockaddr*)&slave_addr_, sizeof(slave_addr_));
        }
    }
    
    void statusCallback() {
        std_msgs::msg::Bool conn;
        conn.data = host_status_.connected && slave_connected_;
        connection_pub_->publish(conn);
        
        std_msgs::msg::String status;
        char buf[200];
        snprintf(buf, sizeof(buf),
                "Host: %s (%.1fV) | Slave: %s (%.1fV, %.2fm)",
                host_status_.connected ? "OK" : "LOST", host_status_.battery_voltage,
                slave_status_.connected ? "OK" : "LOST", slave_status_.battery_voltage,
                slave_status_.obstacle.min_distance);
        status.data = buf;
        status_pub_->publish(status);
        
        RCLCPP_INFO(this->get_logger(), "%s", buf);
    }
    
    size_t getPacketLength(uint8_t type) {
        switch (type) {
            case 0x01: case 0x02: return 16;
            case 0x10: return 200;
            case 0x11: return 28;
            case 0x12: case 0x20: case 0x21: return 14;
            default: return 4;
        }
    }
    
    uint32_t getTimestampMs() {
        return static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count());
    }

private:
    BridgeParams params_;
    std::atomic<bool> running_;
    std::thread serial_thread_, udp_thread_, monitor_thread_;
    
    int serial_fd_ = -1;
    int udp_sock_ = -1;
    struct sockaddr_in slave_addr_;
    socklen_t slave_addr_len_ = 0;
    
    MCUStatus host_status_;
    MCUStatus slave_status_;
    std::atomic<bool> slave_connected_{false};
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connection_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr follow_cmd_sub_;
    
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MicroROSBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
