/**
 * @file apriltag_localization_node.cpp
 * @brief AprilTag从机定位节点 - ROS 2 C++实现
 * 
 * 功能概述：
 * 1. 检测从机上的AprilTag标签
 * 2. 计算从机相对主机的位姿（位置+朝向）
 * 3. 发布跟随指令给从机
 * 4. 支持多标签检测提高鲁棒性
 * 
 * 硬件配置：
 * - 摄像头：Raspberry Pi Camera (CSI接口)
 * - 分辨率：640x480 @ 30fps
 * - 标签：AprilTag 36h11 family
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/getopt.h>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

/**
 * @brief AprilTag定位参数
 */
struct AprilTagParams {
    // 标签参数
    double tag_size = 0.05;         // 标签尺寸(m)
    int tag_family = 0;              // 0: 36h11
    
    // 相机内参（需要标定）
    double fx = 309.0;              // 焦距x
    double fy = 309.0;              // 焦距y
    double cx = 320.0;              // 光心x
    double cy = 240.0;              // 光心y
    
    // 跟随控制参数
    double target_distance = 0.6;   // 目标跟随距离(m)
    double distance_tolerance = 0.1; // 距离容差(m)
    double kp_distance = 0.8;       // 距离控制增益
    double kp_angle = 2.0;          // 角度控制增益
    double max_linear = 0.3;        // 最大线速度(m/s)
    double max_angular = 1.0;       // 最大角速度(rad/s)
    
    // 检测参数
    double decimate = 1.0;          // 图像降采样
    double blur = 0.0;              // 高斯模糊
    int threads = 2;                // 线程数
    int debug = 0;                  // 调试模式
    double refine_edges = 1;        // 边缘精化
};

/**
 * @brief AprilTag检测节点类
 */
class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode() : Node("apriltag_node") {
        // 声明参数
        declareParameters();
        
        // 初始化AprilTag检测器
        initDetector();
        
        // 创建订阅者和发布者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rear/image_raw", rclcpp::QoS(10),
            std::bind(&AprilTagNode::imageCallback, this, std::placeholders::_1));
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/slave_pose", 10);
        
        follow_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/follow_cmd", 10);
        
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/apriltag/debug", 10);
        
        // 创建丢失检测定时器
        lost_timer_ = this->create_wall_timer(
            200ms, std::bind(&AprilTagNode::lostCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "AprilTag node initialized");
        RCLCPP_INFO(this->get_logger(), "Tag size: %.3fm, Target distance: %.2fm",
                   params_.tag_size, params_.target_distance);
    }
    
    ~AprilTagNode() {
        // 释放AprilTag资源
        if (td_) {
            apriltag_detector_destroy(td_);
        }
        if (tf_) {
            tag36h11_destroy(tf_);
        }
    }

private:
    /**
     * @brief 声明ROS参数
     */
    void declareParameters() {
        this->declare_parameter("tag_size", params_.tag_size);
        this->declare_parameter("fx", params_.fx);
        this->declare_parameter("fy", params_.fy);
        this->declare_parameter("cx", params_.cx);
        this->declare_parameter("cy", params_.cy);
        this->declare_parameter("target_distance", params_.target_distance);
        this->declare_parameter("kp_distance", params_.kp_distance);
        this->declare_parameter("kp_angle", params_.kp_angle);
        this->declare_parameter("max_linear", params_.max_linear);
        this->declare_parameter("max_angular", params_.max_angular);
        
        // 获取参数
        params_.tag_size = this->get_parameter("tag_size").as_double();
        params_.fx = this->get_parameter("fx").as_double();
        params_.fy = this->get_parameter("fy").as_double();
        params_.cx = this->get_parameter("cx").as_double();
        params_.cy = this->get_parameter("cy").as_double();
        params_.target_distance = this->get_parameter("target_distance").as_double();
        params_.kp_distance = this->get_parameter("kp_distance").as_double();
        params_.kp_angle = this->get_parameter("kp_angle").as_double();
        params_.max_linear = this->get_parameter("max_linear").as_double();
        params_.max_angular = this->get_parameter("max_angular").as_double();
    }
    
    /**
     * @brief 初始化AprilTag检测器
     */
    void initDetector() {
        // 创建标签族
        tf_ = tag36h11_create();
        
        // 创建检测器
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);
        
        // 设置检测参数
        td_->quad_decimate = params_.decimate;
        td_->quad_sigma = params_.blur;
        td_->nthreads = params_.threads;
        td_->debug = params_.debug;
        td_->refine_edges = params_.refine_edges;
        
        RCLCPP_INFO(this->get_logger(), "AprilTag detector initialized");
    }
    
    /**
     * @brief 图像回调
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 转换为灰度图
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat gray = cv_ptr->image;
        
        // 创建AprilTag图像结构
        image_u8_t im = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
        
        // 检测AprilTag
        zarray_t* detections = apriltag_detector_detect(td_, &im);
        
        int num_detections = zarray_size(detections);
        
        if (num_detections > 0) {
            tag_lost_count_ = 0;
            tag_detected_ = true;
            
            // 处理检测结果
            for (int i = 0; i < num_detections; i++) {
                apriltag_detection_t* det;
                zarray_get(detections, i, &det);
                
                // 估计位姿
                estimatePose(det);
                
                // 只处理第一个检测到的标签
                break;
            }
        } else {
            tag_lost_count_++;
            if (tag_lost_count_ > 3) {
                tag_detected_ = false;
                
                // 发送停止指令
                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.angular.z = 0.0;
                follow_cmd_pub_->publish(stop_cmd);
                
                RCLCPP_WARN(this->get_logger(), "AprilTag lost, stopping slave");
            }
        }
        
        // 释放检测结果
        apriltag_detections_destroy(detections);
        
        last_image_time_ = this->now();
        
        // 计算处理时间
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Detection time: %ld ms, tags found: %d", 
                    duration, num_detections);
    }
    
    /**
     * @brief 估计AprilTag位姿
     */
    void estimatePose(apriltag_detection_t* det) {
        // 获取检测到的角点
        // det->p[0-3] 是四个角点的图像坐标
        
        // 计算标签中心
        double center_x = det->c[0];
        double center_y = det->c[1];
        
        // 计算3D位姿（使用PnP方法简化版）
        // 这里使用简化的单应性矩阵方法
        
        // 标签在相机坐标系中的位置
        // 假设标签平面与相机光轴垂直
        
        // 计算距离（基于标签尺寸和图像尺寸）
        double tag_pixel_size = std::sqrt(
            std::pow(det->p[0][0] - det->p[1][0], 2) +
            std::pow(det->p[0][1] - det->p[1][1], 2));
        
        // 距离 = (实际尺寸 * 焦距) / 像素尺寸
        double distance = (params_.tag_size * params_.fx) / tag_pixel_size;
        
        // 计算相对于图像中心的偏移
        double offset_x = center_x - params_.cx;
        double offset_y = center_y - params_.cy;
        
        // 计算3D位置
        double x = (offset_x * distance) / params_.fx;
        double y = (offset_y * distance) / params_.fy;
        double z = distance;
        
        // 计算偏航角（基于标签的朝向）
        double yaw = calculateYaw(det);
        
        // 发布位姿
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "camera_rear";
        pose_msg.pose.position.x = z;   // 前方距离
        pose_msg.pose.position.y = -x;  // 左右偏移
        pose_msg.pose.position.z = -y;  // 上下偏移
        
        // 设置朝向
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        pose_pub_->publish(pose_msg);
        
        // 计算跟随指令
        computeFollowCommand(z, x, yaw);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Tag %d: distance=%.2fm, offset=%.2fm, yaw=%.1fdeg",
                   det->id, z, x, yaw * 180.0 / M_PI);
    }
    
    /**
     * @brief 计算偏航角
     */
    double calculateYaw(apriltag_detection_t* det) {
        // 使用标签的边界框计算朝向
        // 简化方法：使用标签的对角线方向
        
        double dx = det->p[0][0] - det->p[2][0];
        double dy = det->p[0][1] - det->p[2][1];
        
        return std::atan2(dy, dx);
    }
    
    /**
     * @brief 计算跟随指令
     */
    void computeFollowCommand(double distance, double lateral_offset, double yaw) {
        geometry_msgs::msg::Twist cmd;
        
        // 计算距离误差
        double distance_error = distance - params_.target_distance;
        
        // 如果在容差范围内，不移动
        if (std::abs(distance_error) < params_.distance_tolerance) {
            cmd.linear.x = 0.0;
        } else {
            // 线速度控制
            cmd.linear.x = params_.kp_distance * distance_error;
            cmd.linear.x = std::clamp(cmd.linear.x, -params_.max_linear, params_.max_linear);
        }
        
        // 角速度控制（基于横向偏移和偏航角）
        // 横向偏移转换为角度
        double lateral_angle = std::atan2(lateral_offset, distance);
        
        cmd.angular.z = params_.kp_angle * (lateral_angle + yaw * 0.5);
        cmd.angular.z = std::clamp(cmd.angular.z, -params_.max_angular, params_.max_angular);
        
        // 发布跟随指令
        follow_cmd_pub_->publish(cmd);
        
        RCLCPP_DEBUG(this->get_logger(),
                    "Follow cmd: linear=%.2f, angular=%.2f",
                    cmd.linear.x, cmd.angular.z);
    }
    
    /**
     * @brief 标签丢失回调
     */
    void lostCallback() {
        auto now = this->now();
        if ((now - last_image_time_).seconds() > 0.5 && tag_detected_) {
            // 图像超时
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            follow_cmd_pub_->publish(stop_cmd);
            
            RCLCPP_WARN(this->get_logger(), "Image timeout, stopping slave");
            tag_detected_ = false;
        }
    }
    
    // 成员变量
    AprilTagParams params_;
    apriltag_detector_t* td_ = nullptr;
    tag36h11_family_t* tf_ = nullptr;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr follow_cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr lost_timer_;
    
    int tag_lost_count_ = 0;
    bool tag_detected_ = false;
    rclcpp::Time last_image_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
