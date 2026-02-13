/**
 * @file yolo_detection_node.cpp
 * @brief YOLO目标检测与决策节点 - 云端服务器
 * 
 * 功能概述：
 * 1. 接收RPi推送的视频流
 * 2. 运行YOLO进行实时目标检测
 * 3. 识别语义信息（红绿灯、行人、车辆等）
 * 4. 下发高层决策指令到边缘端
 * 
 * 目标类别：
 * - 交通灯（红/黄/绿）
 * - 行人
 * - 车辆
 * - 交通标志
 * 
 * 性能要求：
 * - 推理帧率 >= 15 FPS
 * - 推理延迟 < 100ms
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <mutex>

// 包含TensorRT或ONNX Runtime头文件
// 这里使用简化的接口

using namespace std::chrono_literals;

/**
 * @brief 检测结果结构
 */
struct Detection {
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect bbox;
    
    // 额外信息
    float center_x;
    float center_y;
    float area;
};

/**
 * @brief 决策指令类型
 */
enum class DecisionType {
    CONTINUE = 0,           // 继续行驶
    STOP = 1,               // 停车
    SLOW_DOWN = 2,          // 减速
    TRAFFIC_LIGHT_WAIT = 3, // 红灯等待
    PEDESTRIAN_AVOID = 4,   // 行人避让
    TURN_LEFT = 5,          // 左转
    TURN_RIGHT = 6          // 右转
};

/**
 * @brief YOLO检测参数
 */
struct YOLOParams {
    // 模型参数
    std::string model_path = "yolov8n.engine";  // TensorRT引擎文件
    int input_width = 640;
    int input_height = 640;
    
    // 检测参数
    float conf_threshold = 0.5f;
    float nms_threshold = 0.4f;
    
    // 感兴趣类别
    std::vector<int> target_classes = {
        0,   // person
        1,   // bicycle
        2,   // car
        3,   // motorcycle
        5,   // bus
        7,   // truck
        9,   // traffic light
        11   // stop sign
    };
    
    // 决策参数
    float danger_distance = 0.3f;   // 危险距离阈值（归一化）
    float warning_distance = 0.5f;  // 警告距离阈值
    float traffic_light_height = 0.3f; // 交通灯高度阈值（归一化）
};

/**
 * @brief YOLO检测节点类
 */
class YOLODetectionNode : public rclcpp::Node {
public:
    YOLODetectionNode() : Node("yolo_detection_node") {
        // 声明参数
        declareParameters();
        
        // 初始化模型
        initModel();
        
        // 创建订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/video_stream", rclcpp::QoS(10),
            std::bind(&YOLODetectionNode::imageCallback, this, std::placeholders::_1));
        
        // 创建发布者
        decision_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/decision_command", 10);
        
        detection_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/yolo/detection_result", 10);
        
        // 创建决策定时器
        decision_timer_ = this->create_wall_timer(
            100ms, std::bind(&YOLODetectionNode::decisionCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "YOLO detection node initialized");
    }
    
    ~YOLODetectionNode() {
        // 释放模型资源
        releaseModel();
    }

private:
    /**
     * @brief 声明ROS参数
     */
    void declareParameters() {
        this->declare_parameter("model_path", params_.model_path);
        this->declare_parameter("conf_threshold", params_.conf_threshold);
        this->declare_parameter("nms_threshold", params_.nms_threshold);
        
        params_.model_path = this->get_parameter("model_path").as_string();
        params_.conf_threshold = this->get_parameter("conf_threshold").as_double();
        params_.nms_threshold = this->get_parameter("nms_threshold").as_double();
    }
    
    /**
     * @brief 初始化模型
     */
    void initModel() {
        // 这里简化实现，实际应加载TensorRT引擎或ONNX模型
        RCLCPP_INFO(this->get_logger(), "Loading model: %s", params_.model_path.c_str());
        
        // 模拟初始化成功
        model_loaded_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Model loaded successfully");
    }
    
    /**
     * @brief 释放模型资源
     */
    void releaseModel() {
        // 释放资源
        model_loaded_ = false;
    }
    
    /**
     * @brief 图像回调
     */
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (!model_loaded_) {
            return;
        }
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 解压缩图像
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
            return;
        }
        
        // 执行检测
        std::vector<Detection> detections = detect(frame);
        
        // 更新检测结果
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            latest_detections_ = detections;
            latest_frame_ = frame.clone();
        }
        
        // 发布检测结果图像
        if (detection_pub_->get_subscription_count() > 0) {
            cv::Mat result_img = drawDetections(frame, detections);
            auto result_msg = cv_bridge::CvImage(
                msg->header, "bgr8", result_img).toImageMsg();
            detection_pub_->publish(*result_msg);
        }
        
        last_image_time_ = this->now();
        
        // 计算处理时间
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();
        
        fps_count_++;
        fps_timer_ += duration;
        if (fps_timer_ >= 1000) {
            RCLCPP_INFO(this->get_logger(), "Detection FPS: %d", fps_count_);
            fps_count_ = 0;
            fps_timer_ = 0;
        }
    }
    
    /**
     * @brief 执行YOLO检测
     */
    std::vector<Detection> detect(const cv::Mat& frame) {
        std::vector<Detection> detections;
        
        // 预处理
        cv::Mat input;
        cv::resize(frame, input, cv::Size(params_.input_width, params_.input_height));
        cv::cvtColor(input, input, cv::COLOR_BGR2RGB);
        
        // 归一化
        input.convertTo(input, CV_32F, 1.0 / 255.0);
        
        // ==========================================
        // 这里是简化的模拟检测，实际应调用推理引擎
        // ==========================================
        
        // 模拟检测交通灯
        // 在实际实现中，这里应该调用TensorRT或ONNX Runtime
        simulateDetection(frame, detections);
        
        // 应用NMS
        applyNMS(detections);
        
        return detections;
    }
    
    /**
     * @brief 模拟检测（用于测试）
     */
    void simulateDetection(const cv::Mat& frame, std::vector<Detection>& detections) {
        // 使用颜色检测模拟交通灯检测
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // 检测红色（红灯）
        cv::Mat red_mask;
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask);
        cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), red_mask);
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 100) {  // 面积阈值
                cv::Rect rect = cv::boundingRect(contour);
                if (rect.y < frame.rows * params_.traffic_light_height) {
                    Detection det;
                    det.class_id = 100;  // 红灯
                    det.class_name = "red_light";
                    det.confidence = 0.9f;
                    det.bbox = rect;
                    det.center_x = rect.x + rect.width / 2.0f;
                    det.center_y = rect.y + rect.height / 2.0f;
                    det.area = area;
                    detections.push_back(det);
                }
            }
        }
        
        // 检测绿色（绿灯）
        cv::Mat green_mask;
        cv::inRange(hsv, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), green_mask);
        
        cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 100) {
                cv::Rect rect = cv::boundingRect(contour);
                if (rect.y < frame.rows * params_.traffic_light_height) {
                    Detection det;
                    det.class_id = 101;  // 绿灯
                    det.class_name = "green_light";
                    det.confidence = 0.9f;
                    det.bbox = rect;
                    det.center_x = rect.x + rect.width / 2.0f;
                    det.center_y = rect.y + rect.height / 2.0f;
                    det.area = area;
                    detections.push_back(det);
                }
            }
        }
    }
    
    /**
     * @brief 应用非极大值抑制
     */
    void applyNMS(std::vector<Detection>& detections) {
        if (detections.empty()) return;
        
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        
        for (const auto& det : detections) {
            boxes.push_back(det.bbox);
            scores.push_back(det.confidence);
        }
        
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, 
                          params_.conf_threshold, 
                          params_.nms_threshold, indices);
        
        std::vector<Detection> filtered;
        for (int idx : indices) {
            filtered.push_back(detections[idx]);
        }
        
        detections = filtered;
    }
    
    /**
     * @brief 决策定时器回调
     */
    void decisionCallback() {
        std::vector<Detection> detections;
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            detections = latest_detections_;
        }
        
        // 分析检测结果，做出决策
        DecisionType decision = analyzeAndDecide(detections);
        
        // 发布决策指令
        std_msgs::msg::Int32 cmd_msg;
        cmd_msg.data = static_cast<int>(decision);
        decision_pub_->publish(cmd_msg);
        
        if (decision != DecisionType::CONTINUE) {
            RCLCPP_INFO(this->get_logger(), "Decision: %d", 
                       static_cast<int>(decision));
        }
    }
    
    /**
     * @brief 分析检测结果并做出决策
     */
    DecisionType analyzeAndDecide(const std::vector<Detection>& detections) {
        bool has_red_light = false;
        bool has_green_light = false;
        bool has_pedestrian = false;
        bool has_obstacle = false;
        
        for (const auto& det : detections) {
            // 检查红灯
            if (det.class_name == "red_light") {
                has_red_light = true;
            }
            // 检查绿灯
            if (det.class_name == "green_light") {
                has_green_light = true;
            }
            // 检查行人
            if (det.class_name == "person") {
                // 计算相对距离（基于bbox大小）
                float normalized_area = det.area / (params_.input_width * params_.input_height);
                if (normalized_area > params_.danger_distance) {
                    has_pedestrian = true;
                }
            }
            // 检查障碍物（车辆等）
            if (det.class_name == "car" || det.class_name == "truck") {
                float normalized_area = det.area / (params_.input_width * params_.input_height);
                if (normalized_area > params_.danger_distance) {
                    has_obstacle = true;
                }
            }
        }
        
        // 决策逻辑（优先级从高到低）
        if (has_pedestrian) {
            return DecisionType::PEDESTRIAN_AVOID;
        }
        
        if (has_red_light) {
            return DecisionType::TRAFFIC_LIGHT_WAIT;
        }
        
        if (has_obstacle) {
            return DecisionType::SLOW_DOWN;
        }
        
        return DecisionType::CONTINUE;
    }
    
    /**
     * @brief 绘制检测结果
     */
    cv::Mat drawDetections(const cv::Mat& frame, const std::vector<Detection>& detections) {
        cv::Mat result = frame.clone();
        
        for (const auto& det : detections) {
            // 选择颜色
            cv::Scalar color;
            if (det.class_name == "red_light") {
                color = cv::Scalar(0, 0, 255);  // 红色
            } else if (det.class_name == "green_light") {
                color = cv::Scalar(0, 255, 0);  // 绿色
            } else if (det.class_name == "person") {
                color = cv::Scalar(255, 0, 0);  // 蓝色
            } else {
                color = cv::Scalar(255, 255, 0);  // 青色
            }
            
            // 绘制边界框
            cv::rectangle(result, det.bbox, color, 2);
            
            // 绘制标签
            std::string label = det.class_name + " " + 
                               std::to_string(static_cast<int>(det.confidence * 100)) + "%";
            cv::putText(result, label, 
                       cv::Point(det.bbox.x, det.bbox.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
        
        return result;
    }
    
    // 成员变量
    YOLOParams params_;
    bool model_loaded_ = false;
    
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr decision_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_pub_;
    rclcpp::TimerBase::SharedPtr decision_timer_;
    
    std::mutex detections_mutex_;
    std::vector<Detection> latest_detections_;
    cv::Mat latest_frame_;
    rclcpp::Time last_image_time_;
    
    int fps_count_ = 0;
    int fps_timer_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YOLODetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
