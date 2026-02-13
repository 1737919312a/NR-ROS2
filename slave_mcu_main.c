/**
 * @file slave_mcu_main.c
 * @brief 从机MCU主程序 - ESP32-S3
 * 
 * 功能概述：
 * 1. 本地激光避障：基于LD06激光雷达的独立避障系统
 *    - 前方0.3m强制停车（优先级最高）
 *    - 左右45度扇区检测
 *    - 本地决策优先于远程指令
 * 2. 跟随执行：接收主机的跟随指令并执行
 * 3. 状态上报：电池电压、避障状态、里程计信息
 * 4. WiFi UDP通信：通过5G CPE连接RPi Micro-ROS Agent
 * 
 * 安全设计原则：
 * - 本地避障逻辑在主循环中实时执行，不可阻塞
 * - WiFi UDP通信可能有延迟，本地决策不依赖网络
 * - 紧急停车响应时间小于100ms
 * - LED指示灯显示避障状态
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
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "nvs_flash.h"
#endif

// Micro-ROS头文件
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <std_msgs/msg/float32.h>
#include <builtin_interfaces/msg/time.h>

// 项目头文件
#include "common_types.h"
#include "pid_controller.h"
#include "ld06_lidar.h"

/* ==================== 日志标签 ==================== */
static const char* TAG = "SLAVE_MCU";

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
#define PIN_LED_STATUS         2     // 状态LED
#define PIN_LED_OBSTACLE       3     // 避障LED（红）
#define PIN_BATTERY_ADC        1

// UART配置
#define UART_LIDAR_NUM         UART_NUM_1
#define UART_LIDAR_TX          8
#define UART_LIDAR_RX          9

// WiFi配置
#define WIFI_SSID              "RobotNetwork"
#define WIFI_PASSWORD          "robot2024"
#define WIFI_CONNECTED_BIT     BIT0

// UDP配置
#define UDP_SERVER_PORT        8888
#define UDP_BUFFER_SIZE        2048

// 避障参数（关键安全参数）
#define EMERGENCY_DISTANCE     0.30f   // 紧急停车距离(m)
#define WARNING_DISTANCE       0.50f   // 警告距离(m)
#define SAFE_FOLLOW_DISTANCE   0.60f   // 安全跟随距离(m)
#define FRONT_FOV              90.0f   // 前方检测视场角(deg)
#define SIDE_FOV               45.0f   // 侧方检测视场角(deg)

// 控制周期
#define CONTROL_PERIOD_MS      10      // 100Hz控制频率（避障要求高频）
#define SENSOR_PERIOD_MS       5       // 200Hz传感器读取
#define ROS_SPIN_PERIOD_MS     20      // 50Hz ROS处理
#define STATUS_PERIOD_MS       100     // 10Hz状态上报
#define LED_BLINK_PERIOD_MS    500     // LED闪烁周期

/* ==================== 全局变量 ==================== */

// 驱动实例
static LD06Driver lidar_driver;
static VelocityController velocity_ctrl;

// 传感器数据
static LidarScan latest_scan;
static ObstacleStatus obstacle_status;
static SemaphoreHandle_t scan_mutex;

// 编码器计数
static volatile int32_t encoder_left_count = 0;
static volatile int32_t encoder_right_count = 0;
static int32_t last_encoder_left = 0;
static int32_t last_encoder_right = 0;

// 速度指令
static Twist2D cmd_vel = {0};
static Twist2D follow_cmd = {0};       // 跟随指令
static uint32_t cmd_vel_timestamp = 0;
static SemaphoreHandle_t cmd_vel_mutex;

// 本地避障状态
static bool local_emergency = false;
static bool local_warning = false;
static bool obstacle_led_state = false;

// 机器人状态
static RobotState robot_state = {0};
static Pose2D odometry = {0};

// WiFi事件组
static EventGroupHandle_t wifi_event_group;

// UDP Socket
static int udp_sock = -1;
struct sockaddr_in server_addr;

// Micro-ROS实体
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rclc_support_t support;

// ROS话题
static rcl_subscription_t cmd_vel_sub;
static rcl_publisher_t status_pub;
static rcl_publisher_t obstacle_pub;
static geometry_msgs__msg__Twist cmd_vel_msg;

/* ==================== 硬件抽象层 ==================== */

uint32_t get_timestamp_ms(void) {
#ifdef ESP_PLATFORM
    return (uint32_t)(esp_timer_get_time() / 1000);
#else
    return 0;
#endif
}

/**
 * @brief 初始化PWM（同主机MCU）
 */
static void pwm_init(void) {
#ifdef ESP_PLATFORM
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    ledc_channel_config_t left_conf = {
        .gpio_num = PIN_MOTOR_PWM_LEFT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&left_conf);
    
    ledc_channel_config_t right_conf = {
        .gpio_num = PIN_MOTOR_PWM_RIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&right_conf);
#endif
    
    ESP_LOGI(TAG, "PWM initialized");
}

/**
 * @brief 设置电机PWM输出
 */
static void motor_set_pwm(float left_pwm, float right_pwm) {
    left_pwm = CLAMP(left_pwm, -100.0f, 100.0f);
    right_pwm = CLAMP(right_pwm, -100.0f, 100.0f);
    
    uint32_t left_duty = (uint32_t)(fabsf(left_pwm) * 255 / 100.0f);
    uint32_t right_duty = (uint32_t)(fabsf(right_pwm) * 255 / 100.0f);
    
#ifdef ESP_PLATFORM
    gpio_set_level(PIN_MOTOR_LEFT_A, left_pwm >= 0 ? 1 : 0);
    gpio_set_level(PIN_MOTOR_LEFT_B, left_pwm >= 0 ? 0 : 1);
    gpio_set_level(PIN_MOTOR_RIGHT_A, right_pwm >= 0 ? 1 : 0);
    gpio_set_level(PIN_MOTOR_RIGHT_B, right_pwm >= 0 ? 0 : 1);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left_duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
#endif
}

/**
 * @brief 紧急停车（本地最高优先级）
 */
static void emergency_stop(void) {
    motor_set_pwm(0, 0);
    velocity_controller_reset(&velocity_ctrl);
    local_emergency = true;
    
    // 点亮红色LED
    gpio_set_level(PIN_LED_OBSTACLE, 1);
    
    ESP_LOGW(TAG, "EMERGENCY STOP! Obstacle detected at %.2fm", 
             obstacle_status.min_distance);
}

/**
 * @brief 清除紧急状态
 */
static void clear_emergency(void) {
    local_emergency = false;
    gpio_set_level(PIN_LED_OBSTACLE, 0);
    ESP_LOGI(TAG, "Emergency cleared, resuming normal operation");
}

/**
 * @brief 设置状态LED
 */
static void set_status_led(bool on) {
    gpio_set_level(PIN_LED_STATUS, on ? 1 : 0);
}

/**
 * @brief 初始化GPIO
 */
static void gpio_init(void) {
#ifdef ESP_PLATFORM
    // 电机和LED引脚
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_LEFT_A) | 
                        (1ULL << PIN_MOTOR_LEFT_B) |
                        (1ULL << PIN_MOTOR_RIGHT_A) | 
                        (1ULL << PIN_MOTOR_RIGHT_B) |
                        (1ULL << PIN_LED_STATUS) |
                        (1ULL << PIN_LED_OBSTACLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // 编码器引脚
    gpio_config_t encoder_conf = {
        .pin_bit_mask = (1ULL << PIN_ENCODER_LEFT_A) | 
                        (1ULL << PIN_ENCODER_LEFT_B) |
                        (1ULL << PIN_ENCODER_RIGHT_A) | 
                        (1ULL << PIN_ENCODER_RIGHT_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&encoder_conf);
#endif
}

/**
 * @brief 编码器中断处理
 */
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    
    if (gpio_num == PIN_ENCODER_LEFT_A) {
        int level = gpio_get_level(PIN_ENCODER_LEFT_B);
        encoder_left_count += level ? -1 : 1;
    } else if (gpio_num == PIN_ENCODER_RIGHT_A) {
        int level = gpio_get_level(PIN_ENCODER_RIGHT_B);
        encoder_right_count += level ? 1 : -1;
    }
}

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
    return (raw / 4095.0f) * 3.3f * 3.0f;
#else
    return 12.0f;
#endif
}

/* ==================== 本地避障逻辑（核心安全功能） ==================== */

/**
 * @brief 执行本地避障检测
 * 
 * 这是系统的核心安全功能，必须在主循环中实时执行。
 * 避障逻辑优先级高于任何远程指令。
 * 
 * @return true 如果需要停车
 */
static bool local_obstacle_check(void) {
    // 定义避障配置
    ObstacleConfig config = {
        .emergency_distance = EMERGENCY_DISTANCE,
        .warning_distance = WARNING_DISTANCE,
        .front_fov = FRONT_FOV,
        .side_fov = SIDE_FOV,
        .max_safe_speed = MOTOR_MAX_SPEED
    };
    
    // 分析障碍物状态
    obstacle_status = ld06_analyze_obstacles(&latest_scan, &config);
    
    // 紧急停车条件：前方0.3m内有障碍物
    if (obstacle_status.min_distance <= EMERGENCY_DISTANCE) {
        obstacle_status.emergency_stop = true;
        return true;  // 需要紧急停车
    }
    
    return false;
}

/**
 * @brief 计算安全速度
 * 
 * 根据障碍物距离动态调整速度，实现平滑减速
 */
static float compute_safe_linear_speed(float desired_speed) {
    float min_dist = obstacle_status.min_distance;
    
    if (min_dist <= EMERGENCY_DISTANCE) {
        return 0.0f;
    } else if (min_dist <= WARNING_DISTANCE) {
        // 线性减速
        float ratio = (min_dist - EMERGENCY_DISTANCE) / 
                      (WARNING_DISTANCE - EMERGENCY_DISTANCE);
        return desired_speed * ratio * ratio;  // 平方减速更平滑
    }
    
    return desired_speed;
}

/* ==================== WiFi和UDP通信 ==================== */

#ifdef ESP_PLATFORM
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && 
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
#endif

static void wifi_init_sta(void) {
#ifdef ESP_PLATFORM
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization finished");
#endif
}

/* ==================== Micro-ROS ==================== */

static void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = 
        (const geometry_msgs__msg__Twist*)msgin;
    
    if (xSemaphoreTake(cmd_vel_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // 注意：本地避障可能覆盖此指令
        follow_cmd.linear_x = (float)msg->linear.x;
        follow_cmd.angular_z = (float)msg->angular.z;
        follow_cmd.timestamp = get_timestamp_ms();
        cmd_vel_timestamp = follow_cmd.timestamp;
        xSemaphoreGive(cmd_vel_mutex);
    }
    
    ESP_LOGD(TAG, "Follow cmd: linear=%.2f angular=%.2f", 
             msg->linear.x, msg->angular.z);
}

static bool microros_init(void) {
    // 使用UDP传输
    rmw_uros_set_custom_transport(
        false,
        (void*)"192.168.1.100",  // RPi IP (需要配置)
        esp32_udp_open,
        esp32_udp_close,
        esp32_udp_write,
        esp32_udp_read,
        esp32_udp_close
    );
    
    allocator = rcl_get_default_allocator();
    
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init support: %d", ret);
        return false;
    }
    
    ret = rclc_node_init_default(&node, "slave_mcu", "", &support);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init node: %d", ret);
        return false;
    }
    
    ret = rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "follow_cmd"
    );
    
    ret = rclc_publisher_init_default(
        &obstacle_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "obstacle_distance"
    );
    
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    ret = rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA
    );
    
    ESP_LOGI(TAG, "Micro-ROS initialized");
    return true;
}

/* ==================== 任务函数 ==================== */

/**
 * @brief 本地避障检测任务（最高优先级）
 * 
 * 该任务以100Hz频率执行，确保避障响应时间小于100ms
 */
static void obstacle_task(void* arg) {
    ESP_LOGI(TAG, "Obstacle detection task started (100Hz)");
    
    while (1) {
        // 处理激光雷达数据
        if (ld06_process(&lidar_driver)) {
            if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ld06_get_scan(&lidar_driver, &latest_scan);
                xSemaphoreGive(scan_mutex);
            }
            
            // 执行本地避障检测
            bool need_stop = local_obstacle_check();
            
            if (need_stop && !local_emergency) {
                emergency_stop();
            } else if (!need_stop && local_emergency) {
                // 清除紧急状态
                clear_emergency();
            }
            
            // 更新警告LED（闪烁）
            if (obstacle_status.obstacle_front && !local_emergency) {
                obstacle_led_state = !obstacle_led_state;
                gpio_set_level(PIN_LED_OBSTACLE, obstacle_led_state);
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
    bool led_state = false;
    uint32_t led_timer = 0;
    
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
                        ENCODER_PULSES_PER_REV, &odometry);
        
        // 状态LED闪烁（表示系统运行）
        if (now - led_timer > LED_BLINK_PERIOD_MS) {
            led_state = !led_state;
            set_status_led(led_state);
            led_timer = now;
        }
        
        // 紧急停车状态：直接停止电机
        if (local_emergency) {
            motor_set_pwm(0, 0);
            velocity_controller_reset(&velocity_ctrl);
            goto control_end;
        }
        
        // 检查指令超时
        bool cmd_valid = (now - cmd_vel_timestamp) < CMD_VEL_TIMEOUT_MS;
        
        if (!cmd_valid) {
            // 指令超时，逐渐减速停车
            velocity_controller_set_target(&velocity_ctrl, 0, 0);
        } else {
            // 正常跟随控制
            float linear = follow_cmd.linear_x;
            float angular = follow_cmd.angular_z;
            
            // 根据本地避障状态限速
            linear = compute_safe_linear_speed(linear);
            
            // 如果前方有障碍物，调整角速度进行避让
            if (obstacle_status.obstacle_left && !obstacle_status.obstacle_right) {
                angular = MAX(angular, 0.3f);  // 向右转
            } else if (obstacle_status.obstacle_right && !obstacle_status.obstacle_left) {
                angular = MIN(angular, -0.3f);  // 向左转
            }
            
            velocity_controller_set_target(&velocity_ctrl, linear, angular);
        }
        
        // 更新PID控制器
        velocity_controller_update(&velocity_ctrl, delta_left, delta_right, dt);
        
        // 获取并应用PWM输出
        velocity_controller_get_pwm(&velocity_ctrl, &left_pwm, &right_pwm);
        motor_set_pwm(left_pwm, right_pwm);
        
control_end:
        // 更新机器人状态
        robot_state.pose = odometry;
        robot_state.obstacle = obstacle_status;
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS_SPIN_PERIOD_MS));
        
        vTaskDelay(pdMS_TO_TICKS(ROS_SPIN_PERIOD_MS));
    }
}

/**
 * @brief 状态上报任务
 */
static void status_task(void* arg) {
    ESP_LOGI(TAG, "Status task started");
    
    std_msgs__msg__Float32 obstacle_msg;
    
    while (1) {
        // 上报障碍物距离
        obstacle_msg.data = obstacle_status.min_distance;
        rcl_publish(&obstacle_pub, &obstacle_msg, NULL);
        
        ESP_LOGD(TAG, "Status: pose=(%.2f, %.2f, %.1f°) obstacle=%.2fm bat=%.1fV",
                 odometry.x, odometry.y, RAD_TO_DEG(odometry.theta),
                 obstacle_status.min_distance, robot_state.battery_voltage);
        
        vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));
    }
}

/* ==================== 主函数 ==================== */

void app_main(void) {
    ESP_LOGI(TAG, "Slave MCU starting...");
    ESP_LOGI(TAG, "Firmware version: 1.0.0");
    ESP_LOGI(TAG, "Safety: Local obstacle avoidance ENABLED");
    
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
                              1.5f, 0.1f, 0.05f);
    
    // 初始化WiFi
    wifi_init_sta();
    
    // 等待WiFi连接
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected!");
    
    // 等待Micro-ROS Agent
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
    
    // 创建任务（避障任务使用最高优先级）
    xTaskCreatePinnedToCore(obstacle_task, "obstacle", 4096, NULL, 7, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(ros_task, "ros", 8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(status_task, "status", 4096, NULL, 3, NULL, 1);
    
    ESP_LOGI(TAG, "All tasks created, system running");
    ESP_LOGI(TAG, "Local obstacle avoidance: ACTIVE");
    ESP_LOGI(TAG, "Emergency distance: %.2fm", EMERGENCY_DISTANCE);
    ESP_LOGI(TAG, "Warning distance: %.2fm", WARNING_DISTANCE);
    
    // 主循环监控
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 打印状态
        ESP_LOGI(TAG, "Status: Emergency=%d MinDist=%.2fm Bat=%.1fV",
                 local_emergency, obstacle_status.min_distance,
                 robot_state.battery_voltage);
    }
}
