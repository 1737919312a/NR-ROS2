/**
 * @file host_mcu_main.c
 * @brief 主机MCU主程序 - ESP32-S3
 * 
 * 功能概述：
 * 1. 传感器融合：LD06激光雷达 + 毫米波雷达数据处理
 * 2. 电机PID控制：接收RPi的/cmd_vel指令，执行闭环控制
 * 3. Micro-ROS通信：通过USB串口与RPi Agent连接
 * 4. 状态监控：电池电压、温度、错误状态上报
 * 
 * 硬件配置：
 * - ESP32-S3-WROOM-1 (N8R8)
 * - LD06激光雷达 (UART1)
 * - 毫米波雷达 (UART2)
 * - 编码器电机 x2 (GPIO中断)
 * - TB6612FNG电机驱动
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ESP-IDF头文件
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"
#endif

// Micro-ROS头文件
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/string.h>
#include <builtin_interfaces/msg/time.h>

// 项目头文件
#include "common_types.h"
#include "pid_controller.h"
#include "ld06_lidar.h"

/* ==================== 日志标签 ==================== */
static const char* TAG = "HOST_MCU";

/* ==================== 系统参数配置 ==================== */

// 引脚定义
#define PIN_MOTOR_LEFT_A       4
#define PIN_MOTOR_LEFT_B       5
#define PIN_MOTOR_RIGHT_A      6
#define PIN_MOTOR_RIGHT_B      7
#define PIN_MOTOR_PWM_LEFT     15
#define PIN_MOTOR_PWM_RIGHT    16
#define PIN_ENCODER_LEFT_A     18
#define PIN_ENCODER_LEFT_B     19
#define PIN_ENCODER_RIGHT_A    20
#define PIN_ENCODER_RIGHT_B    21
#define PIN_BATTERY_ADC        1   // ADC1_CH0

// UART配置
#define UART_LIDAR_NUM         UART_NUM_1
#define UART_LIDAR_TX          8
#define UART_LIDAR_RX          9
#define UART_RADAR_NUM         UART_NUM_2
#define UART_RADAR_TX          10
#define UART_RADAR_RX          11
#define UART_MICROROS_NUM      UART_NUM_0  // USB Serial

// PWM配置
#define PWM_TIMER              LEDC_TIMER_0
#define PWM_MODE               LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL_LEFT       LEDC_CHANNEL_0
#define PWM_CHANNEL_RIGHT      LEDC_CHANNEL_1
#define PWM_DUTY_MAX           255

// 控制周期
#define CONTROL_PERIOD_MS      20    // 50Hz控制频率
#define SENSOR_PERIOD_MS       10    // 100Hz传感器读取
#define ROS_SPIN_PERIOD_MS     10    // 100Hz ROS处理
#define STATUS_PERIOD_MS       100   // 10Hz状态上报

/* ==================== 全局变量 ==================== */

// 驱动实例
static LD06Driver lidar_driver;
static VelocityController velocity_ctrl;

// 传感器数据
static LidarScan latest_scan;
static RadarData latest_radar;
static ObstacleStatus obstacle_status;
static SemaphoreHandle_t scan_mutex;

// 编码器计数
static volatile int32_t encoder_left_count = 0;
static volatile int32_t encoder_right_count = 0;
static int32_t last_encoder_left = 0;
static int32_t last_encoder_right = 0;

// 速度指令
static Twist2D cmd_vel = {0};
static uint32_t cmd_vel_timestamp = 0;
static SemaphoreHandle_t cmd_vel_mutex;

// 机器人状态
static RobotState robot_state = {0};
static SystemConfig sys_config = {0};

// 看门狗
static uint32_t last_heartbeat = 0;
static bool emergency_active = false;

// Micro-ROS实体
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rclc_support_t support;

// ROS话题
static rcl_subscription_t cmd_vel_sub;
static rcl_publisher_t laser_pub;
static rcl_publisher_t status_pub;
static geometry_msgs__msg__Twist cmd_vel_msg;
static sensor_msgs__msg__LaserScan laser_msg;

/* ==================== 硬件抽象层 ==================== */

uint32_t get_timestamp_ms(void) {
#ifdef ESP_PLATFORM
    return (uint32_t)(esp_timer_get_time() / 1000);
#else
    return 0;
#endif
}

/**
 * @brief 初始化PWM
 */
static void pwm_init(void) {
#ifdef ESP_PLATFORM
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    ledc_channel_config_t left_conf = {
        .gpio_num = PIN_MOTOR_PWM_LEFT,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_LEFT,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&left_conf);
    
    ledc_channel_config_t right_conf = {
        .gpio_num = PIN_MOTOR_PWM_RIGHT,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_RIGHT,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&right_conf);
#endif
    
    ESP_LOGI(TAG, "PWM initialized at %d Hz", MOTOR_PWM_FREQ);
}

/**
 * @brief 设置电机PWM输出
 */
static void motor_set_pwm(float left_pwm, float right_pwm) {
    // 限制范围
    left_pwm = CLAMP(left_pwm, -100.0f, 100.0f);
    right_pwm = CLAMP(right_pwm, -100.0f, 100.0f);
    
    // 转换为PWM占空比
    uint32_t left_duty = (uint32_t)(fabsf(left_pwm) * PWM_DUTY_MAX / 100.0f);
    uint32_t right_duty = (uint32_t)(fabsf(right_pwm) * PWM_DUTY_MAX / 100.0f);
    
#ifdef ESP_PLATFORM
    // 设置方向引脚
    gpio_set_level(PIN_MOTOR_LEFT_A, left_pwm >= 0 ? 1 : 0);
    gpio_set_level(PIN_MOTOR_LEFT_B, left_pwm >= 0 ? 0 : 1);
    gpio_set_level(PIN_MOTOR_RIGHT_A, right_pwm >= 0 ? 1 : 0);
    gpio_set_level(PIN_MOTOR_RIGHT_B, right_pwm >= 0 ? 0 : 1);
    
    // 设置PWM占空比
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_LEFT, left_duty);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_RIGHT, right_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_LEFT);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_RIGHT);
#endif
}

/**
 * @brief 紧急停车
 */
static void emergency_stop(void) {
    motor_set_pwm(0, 0);
    velocity_controller_reset(&velocity_ctrl);
    emergency_active = true;
    ESP_LOGW(TAG, "Emergency stop activated!");
}

/**
 * @brief 初始化GPIO
 */
static void gpio_init(void) {
#ifdef ESP_PLATFORM
    // 电机方向引脚
    gpio_config_t motor_io = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_LEFT_A) | 
                        (1ULL << PIN_MOTOR_LEFT_B) |
                        (1ULL << PIN_MOTOR_RIGHT_A) | 
                        (1ULL << PIN_MOTOR_RIGHT_B),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&motor_io);
    
    // 编码器引脚
    gpio_config_t encoder_io = {
        .pin_bit_mask = (1ULL << PIN_ENCODER_LEFT_A) | 
                        (1ULL << PIN_ENCODER_LEFT_B) |
                        (1ULL << PIN_ENCODER_RIGHT_A) | 
                        (1ULL << PIN_ENCODER_RIGHT_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&encoder_io);
#endif
}

/**
 * @brief 编码器中断处理函数
 */
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    
    // 简化的编码器计数（只检测A相上升沿）
    if (gpio_num == PIN_ENCODER_LEFT_A) {
        int level = gpio_get_level(PIN_ENCODER_LEFT_B);
        if (level) {
            encoder_left_count--;
        } else {
            encoder_left_count++;
        }
    } else if (gpio_num == PIN_ENCODER_RIGHT_A) {
        int level = gpio_get_level(PIN_ENCODER_RIGHT_B);
        if (level) {
            encoder_right_count++;
        } else {
            encoder_right_count--;
        }
    }
}

/**
 * @brief 初始化编码器中断
 */
static void encoder_init(void) {
#ifdef ESP_PLATFORM
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENCODER_LEFT_A, encoder_isr_handler, 
                         (void*)PIN_ENCODER_LEFT_A);
    gpio_isr_handler_add(PIN_ENCODER_RIGHT_A, encoder_isr_handler,
                         (void*)PIN_ENCODER_RIGHT_A);
#endif
}

/**
 * @brief 读取电池电压
 */
static float read_battery_voltage(void) {
#ifdef ESP_PLATFORM
    int raw = adc1_get_raw(PIN_BATTERY_ADC);
    // 假设分压比为1:3，参考电压3.3V
    return (raw / 4095.0f) * 3.3f * 3.0f;
#else
    return 12.0f;  // 默认值
#endif
}

/* ==================== Micro-ROS回调函数 ==================== */

/**
 * @brief cmd_vel订阅回调
 */
static void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = 
        (const geometry_msgs__msg__Twist*)msgin;
    
    if (xSemaphoreTake(cmd_vel_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cmd_vel.linear_x = (float)msg->linear.x;
        cmd_vel.linear_y = (float)msg->linear.y;
        cmd_vel.angular_z = (float)msg->angular.z;
        cmd_vel.timestamp = get_timestamp_ms();
        cmd_vel_timestamp = cmd_vel.timestamp;
        xSemaphoreGive(cmd_vel_mutex);
    }
    
    ESP_LOGD(TAG, "cmd_vel: linear=%.2f angular=%.2f", 
             msg->linear.x, msg->angular.z);
}

/* ==================== Micro-ROS初始化 ==================== */

static bool microros_init(void) {
    // 设置串口传输
    rmw_uros_set_custom_transport(
        true,
        NULL,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read,
        esp32_serial_close
    );
    
    allocator = rcl_get_default_allocator();
    
    // 初始化支持
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init support: %d", ret);
        return false;
    }
    
    // 创建节点
    ret = rclc_node_init_default(&node, "host_mcu", "", &support);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init node: %d", ret);
        return false;
    }
    
    // 创建订阅者
    ret = rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init cmd_vel subscriber: %d", ret);
        return false;
    }
    
    // 创建发布者
    ret = rclc_publisher_init_default(
        &laser_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"
    );
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init laser publisher: %d", ret);
        return false;
    }
    
    // 初始化执行器
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init executor: %d", ret);
        return false;
    }
    
    ret = rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add subscription: %d", ret);
        return false;
    }
    
    ESP_LOGI(TAG, "Micro-ROS initialized successfully");
    return true;
}

/* ==================== 任务函数 ==================== */

/**
 * @brief 传感器读取任务
 */
static void sensor_task(void* arg) {
    ESP_LOGI(TAG, "Sensor task started");
    
    ObstacleConfig obstacle_cfg = {
        .emergency_distance = EMERGENCY_STOP_DISTANCE,
        .warning_distance = COLLISION_AVOID_DISTANCE,
        .front_fov = 60.0f,   // 前方60度
        .side_fov = 30.0f,    // 侧方30度
        .max_safe_speed = MOTOR_MAX_SPEED
    };
    
    while (1) {
        // 处理激光雷达数据
        if (ld06_process(&lidar_driver)) {
            if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ld06_get_scan(&lidar_driver, &latest_scan);
                
                // 分析障碍物
                obstacle_status = ld06_analyze_obstacles(&latest_scan, &obstacle_cfg);
                
                xSemaphoreGive(scan_mutex);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/**
 * @brief 电机控制任务
 */
static void control_task(void* arg) {
    ESP_LOGI(TAG, "Control task started");
    
    float left_pwm, right_pwm;
    int32_t delta_left, delta_right;
    uint32_t last_time = get_timestamp_ms();
    
    while (1) {
        uint32_t now = get_timestamp_ms();
        float dt = (now - last_time) / 1000.0f;
        last_time = now;
        
        // 获取编码器增量
        delta_left = encoder_left_count - last_encoder_left;
        delta_right = encoder_right_count - last_encoder_right;
        last_encoder_left = encoder_left_count;
        last_encoder_right = encoder_right_count;
        
        // 更新里程计
        odometry_update(delta_left, delta_right, 
                        &velocity_ctrl.kinematics,
                        ENCODER_PULSES_PER_REV,
                        &robot_state.pose);
        
        // 检查cmd_vel超时
        bool cmd_valid = (now - cmd_vel_timestamp) < CMD_VEL_TIMEOUT_MS;
        
        // 检查紧急停车条件
        if (obstacle_status.emergency_stop) {
            emergency_stop();
            goto control_end;
        }
        
        // 安全降级逻辑
        if (!cmd_valid || emergency_active) {
            // 逐渐减速停车
            velocity_controller_set_target(&velocity_ctrl, 0, 0);
        } else {
            // 正常控制
            float linear = cmd_vel.linear_x;
            float angular = cmd_vel.angular_z;
            
            // 根据障碍物状态限速
            if (obstacle_status.obstacle_front) {
                linear = MIN(linear, obstacle_status.safe_speed);
            }
            
            velocity_controller_set_target(&velocity_ctrl, linear, angular);
            emergency_active = false;
        }
        
        // 更新PID控制器
        velocity_controller_update(&velocity_ctrl, delta_left, delta_right, dt);
        
        // 获取PWM输出
        velocity_controller_get_pwm(&velocity_ctrl, &left_pwm, &right_pwm);
        
        // 应用到电机
        if (velocity_ctrl.enabled) {
            motor_set_pwm(left_pwm, right_pwm);
        } else {
            motor_set_pwm(0, 0);
        }
        
control_end:
        // 更新状态
        robot_state.velocity.linear_x = (delta_left + delta_right) * 
            velocity_ctrl.kinematics.wheel_radius / 
            (2.0f * ENCODER_PULSES_PER_REV * 2 * 3.14159f) / dt;
        robot_state.battery_voltage = read_battery_voltage();
        robot_state.timestamp = now;
        
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/**
 * @brief ROS处理任务
 */
static void ros_task(void* arg) {
    ESP_LOGI(TAG, "ROS task started");
    
    while (1) {
        // 执行ROS回调
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS_SPIN_PERIOD_MS));
        
        vTaskDelay(pdMS_TO_TICKS(ROS_SPIN_PERIOD_MS));
    }
}

/**
 * @brief 状态发布任务
 */
static void status_task(void* arg) {
    ESP_LOGI(TAG, "Status task started");
    
    while (1) {
        // 发布激光扫描数据
        if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // 填充LaserScan消息
            laser_msg.header.stamp.sec = latest_scan.timestamp / 1000;
            laser_msg.header.stamp.nanosec = (latest_scan.timestamp % 1000) * 1000000;
            laser_msg.header.frame_id.data = "laser_frame";
            laser_msg.angle_min = 0.0f;
            laser_msg.angle_max = 2 * 3.14159265f;
            laser_msg.angle_increment = 3.14159265f * 2.0f / LD06_POINTS_PER_SCAN;
            laser_msg.time_increment = 1.0f / LD06_SCAN_FREQ / LD06_POINTS_PER_SCAN;
            laser_msg.scan_time = 1.0f / LD06_SCAN_FREQ;
            laser_msg.range_min = LD06_MIN_DISTANCE;
            laser_msg.range_max = LD06_MAX_DISTANCE;
            
            // 填充距离数据
            for (int i = 0; i < LD06_POINTS_PER_SCAN && i < laser_msg.ranges.capacity; i++) {
                laser_msg.ranges.data[i] = latest_scan.points[i].valid ? 
                    latest_scan.points[i].distance : NAN;
            }
            laser_msg.ranges.size = MIN(LD06_POINTS_PER_SCAN, laser_msg.ranges.capacity);
            
            rcl_ret_t ret = rcl_publish(&laser_pub, &laser_msg, NULL);
            if (ret != RCL_RET_OK) {
                ESP_LOGW(TAG, "Failed to publish laser scan: %d", ret);
            }
            
            xSemaphoreGive(scan_mutex);
        }
        
        // 更新心跳
        last_heartbeat = get_timestamp_ms();
        
        vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));
    }
}

/* ==================== 主函数 ==================== */

void app_main(void) {
    ESP_LOGI(TAG, "Host MCU starting...");
    ESP_LOGI(TAG, "Firmware version: 1.0.0");
    
    // 初始化NVS
#ifdef ESP_PLATFORM
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 初始化ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(PIN_BATTERY_ADC, ADC_ATTEN_DB_11);
#endif
    
    // 创建互斥锁
    scan_mutex = xSemaphoreCreateMutex();
    cmd_vel_mutex = xSemaphoreCreateMutex();
    
    // 初始化硬件
    gpio_init();
    pwm_init();
    encoder_init();
    
    // 初始化激光雷达
    ld06_init(&lidar_driver, UART_LIDAR_NUM, UART_LIDAR_TX, UART_LIDAR_RX);
    ld06_start(&lidar_driver);
    
    // 初始化速度控制器
    velocity_controller_init(&velocity_ctrl, 
                              WHEEL_BASE, 
                              WHEEL_DIAMETER / 2.0f,
                              1.5f,   // Kp
                              0.1f,   // Ki
                              0.05f); // Kd
    
    // 等待Micro-ROS Agent连接
    ESP_LOGI(TAG, "Waiting for Micro-ROS Agent...");
    while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Micro-ROS Agent connected!");
    
    // 初始化Micro-ROS
    if (!microros_init()) {
        ESP_LOGE(TAG, "Micro-ROS initialization failed!");
        esp_restart();
    }
    
    // 初始化LaserScan消息
    laser_msg.ranges.data = (float*)malloc(LD06_POINTS_PER_SCAN * sizeof(float));
    laser_msg.ranges.capacity = LD06_POINTS_PER_SCAN;
    laser_msg.ranges.size = 0;
    laser_msg.intensities.data = (float*)malloc(LD06_POINTS_PER_SCAN * sizeof(float));
    laser_msg.intensities.capacity = LD06_POINTS_PER_SCAN;
    laser_msg.intensities.size = 0;
    
    // 创建任务
    xTaskCreatePinnedToCore(sensor_task, "sensor", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(ros_task, "ros", 8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(status_task, "status", 4096, NULL, 3, NULL, 1);
    
    ESP_LOGI(TAG, "All tasks created, system running");
    
    // 主循环监控
    while (1) {
        // 检查看门狗
        uint32_t now = get_timestamp_ms();
        if (now - last_heartbeat > WATCHDOG_TIMEOUT_MS) {
            ESP_LOGW(TAG, "Watchdog timeout, emergency stop");
            emergency_stop();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
