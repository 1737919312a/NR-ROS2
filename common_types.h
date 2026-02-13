/**
 * @file common_types.h
 * @brief ROS2云边端协同系统 - 公共类型定义
 * 
 * 该头文件定义了系统中所有模块共用的数据类型、常量和宏定义。
 * 确保跨平台兼容性（ESP32-S3, Raspberry Pi, Linux服务器）
 */

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 系统常量定义 ==================== */

// 串口通信参数
#define SERIAL_BAUD_RATE_HIGH    921600    // 高速串口波特率
#define SERIAL_BAUD_RATE_NORMAL  115200    // 普通串口波特率
#define SERIAL_BUFFER_SIZE       512       // 串口缓冲区大小
#define SERIAL_TIMEOUT_MS        100       // 串口超时时间(ms)

// UDP通信参数
#define UDP_PORT_DEFAULT         8888      // Micro-ROS UDP端口
#define UDP_BUFFER_SIZE          2048      // UDP缓冲区大小
#define UDP_TIMEOUT_MS           500       // UDP超时时间(ms)

// 传感器参数
#define LD06_SCAN_FREQ           12        // LD06激光雷达扫描频率(Hz)
#define LD06_POINTS_PER_SCAN     360       // 每圈扫描点数
#define LD06_ANGLE_STEP          1.0f      // 角度步进(度)
#define LD06_MAX_DISTANCE        12.0f     // 最大检测距离(m)
#define LD06_MIN_DISTANCE        0.05f     // 最小检测距离(m)
#define LD06_PACKET_SIZE         46        // LD06数据包长度(字节)

// 运动控制参数
#define MOTOR_PWM_FREQ           20000     // PWM频率(Hz) - 高于可闻频率
#define MOTOR_PWM_RESOLUTION     8         // PWM分辨率(位)
#define MOTOR_MAX_SPEED          2.0f      // 最大速度(m/s)
#define MOTOR_MAX_ANGULAR        3.14f     // 最大角速度(rad/s)
#define ENCODER_PULSES_PER_REV   1320      // 编码器每转脉冲数
#define WHEEL_DIAMETER           0.065f    // 轮子直径(m)
#define WHEEL_BASE               0.15f     // 轮距(m)

// 安全参数
#define WATCHDOG_TIMEOUT_MS      500       // 看门狗超时时间(ms)
#define CMD_VEL_TIMEOUT_MS       200       // 速度指令超时时间(ms)
#define EMERGENCY_STOP_DISTANCE  0.30f     // 紧急停车距离(m)
#define SAFE_FOLLOW_DISTANCE     0.50f     // 安全跟随距离(m)
#define COLLISION_AVOID_DISTANCE 0.50f     // 避障距离阈值(m)

// 视觉参数
#define CAMERA_WIDTH             640       // 摄像头分辨率宽度
#define CAMERA_HEIGHT            480       // 摄像头分辨率高度
#define CAMERA_FPS               30        // 帧率
#define LANE_DETECT_ROI_TOP      300       // 车道检测ROI顶部
#define LANE_DETECT_ROI_BOTTOM   480       // 车道检测ROI底部

// AprilTag参数
#define APRILTAG_FAMILY          0         // 36h11 family
#define APRILTAG_SIZE            0.05f     // 标签尺寸(m)
#define APRILTAG_MAX_DISTANCE    2.0f      // 最大检测距离(m)

/* ==================== 数据结构定义 ==================== */

/**
 * @brief 2D向量结构
 */
typedef struct {
    float x;
    float y;
} Vector2D;

/**
 * @brief 3D位姿结构
 */
typedef struct {
    float x;           // x坐标(m)
    float y;           // y坐标(m)
    float z;           // z坐标(m)
    float roll;        // 翻滚角(rad)
    float pitch;       // 俯仰角(rad)
    float yaw;         // 偏航角(rad)
} Pose3D;

/**
 * @brief 2D位姿结构（平面运动）
 */
typedef struct {
    float x;           // x坐标(m)
    float y;           // y坐标(m)
    float theta;       // 朝向角(rad)
} Pose2D;

/**
 * @brief 速度指令结构
 */
typedef struct {
    float linear_x;    // 线速度(m/s) - 前进
    float linear_y;    // 线速度(m/s) - 侧向（全向轮用）
    float angular_z;   // 角速度(rad/s) - 旋转
    uint32_t timestamp; // 时间戳(ms)
} Twist2D;

/**
 * @brief 编码器数据结构
 */
typedef struct {
    int32_t left;      // 左轮脉冲计数
    int32_t right;     // 右轮脉冲计数
    uint32_t timestamp; // 时间戳(ms)
} EncoderData;

/**
 * @brief 电机状态结构
 */
typedef struct {
    float left_speed;   // 左轮速度(m/s)
    float right_speed;  // 右轮速度(m/s)
    float left_pwm;     // 左轮PWM占空比(%)
    float right_pwm;    // 右轮PWM占空比(%)
    bool left_dir;      // 左轮方向 (true:前进)
    bool right_dir;     // 右轮方向 (true:前进)
} MotorState;

/**
 * @brief 激光雷达单点数据
 */
typedef struct {
    float angle;        // 角度(deg)
    float distance;     // 距离(m)
    float intensity;    // 强度
    bool valid;         // 数据有效性
} LidarPoint;

/**
 * @brief 激光雷达扫描数据
 */
typedef struct {
    LidarPoint points[LD06_POINTS_PER_SCAN];
    uint16_t point_count;
    uint32_t timestamp;
    float scan_time;    // 扫描时间(s)
} LidarScan;

/**
 * @brief 毫米波雷达目标数据
 */
typedef struct {
    float distance;     // 距离(m)
    float speed;        // 相对速度(m/s)
    float angle;        // 角度(deg)
    float rcs;          // 雷达散射截面(dBsm)
    uint8_t id;         // 目标ID
} RadarTarget;

/**
 * @brief 毫米波雷达数据
 */
typedef struct {
    RadarTarget targets[8];
    uint8_t target_count;
    uint32_t timestamp;
} RadarData;

/**
 * @brief 障碍物信息
 */
typedef struct {
    float distance;     // 最近距离(m)
    float angle;        // 角度(deg)
    float speed;        // 相对速度(m/s)
    uint8_t source;     // 来源 (0:激光, 1:毫米波, 2:融合)
    bool active;        // 是否有效
} ObstacleInfo;

/**
 * @brief 避障状态
 */
typedef struct {
    bool emergency_stop;        // 紧急停车
    bool obstacle_front;        // 前方有障碍
    bool obstacle_left;         // 左前方有障碍
    bool obstacle_right;        // 右前方有障碍
    float min_distance;         // 最近障碍距离
    float safe_speed;           // 安全速度建议
    uint32_t timestamp;
} ObstacleStatus;

/**
 * @brief AprilTag检测结果
 */
typedef struct {
    uint16_t id;                // 标签ID
    Pose3D pose;                // 位姿
    float center_x;             // 图像中心x
    float center_y;             // 图像中心y
    float corners[8];           // 四角坐标 (x0,y0,x1,y1,...)
    bool detected;              // 检测成功标志
    uint32_t timestamp;
} AprilTagDetection;

/**
 * @brief 车道线检测结果
 */
typedef struct {
    float left_line[4];         // 左车道线参数 [a,b,c,d] 或 [起点x,y,终点x,y]
    float right_line[4];        // 右车道线参数
    float center_offset;        // 相对车道中心偏移(m)
    float heading_error;        // 航向误差(rad)
    bool left_detected;         // 左车道线检测成功
    bool right_detected;        // 右车道线检测成功
    bool lane_valid;            // 车道有效
    uint32_t timestamp;
} LaneDetection;

/**
 * @brief YOLO检测结果
 */
typedef struct {
    uint16_t class_id;          // 类别ID
    float confidence;           // 置信度
    float bbox[4];              // 边界框 [x,y,w,h]
    char class_name[32];        // 类别名称
} YOLODetection;

/**
 * @brief 云端决策指令
 */
typedef struct {
    uint8_t command;            // 命令类型
    uint8_t priority;           // 优先级
    float param1;               // 参数1
    float param2;               // 参数2
    uint32_t sequence;          // 序列号
    uint32_t timestamp;         // 时间戳
    uint16_t checksum;          // 校验和
} DecisionCommand;

// 决策命令类型定义
#define CMD_NONE                0
#define CMD_CONTINUE            1
#define CMD_STOP                2
#define CMD_SLOW_DOWN           3
#define CMD_TURN_LEFT           4
#define CMD_TURN_RIGHT          5
#define CMD_UTURN               6
#define CMD_PEDESTRIAN_AVOID    7
#define CMD_TRAFFIC_LIGHT_WAIT  8
#define CMD_FOLLOW_START        9
#define CMD_FOLLOW_STOP         10

/**
 * @brief 机器人状态
 */
typedef struct {
    Pose2D pose;                // 当前位姿
    Twist2D velocity;           // 当前速度
    Twist2D cmd_vel;            // 目标速度
    ObstacleStatus obstacle;    // 障碍物状态
    float battery_voltage;      // 电池电压
    uint8_t state;              // 运行状态
    uint8_t error_code;         // 错误码
    uint32_t timestamp;
} RobotState;

// 机器人状态定义
#define STATE_IDLE              0
#define STATE_MANUAL            1
#define STATE_AUTO              2
#define STATE_FOLLOW            3
#define STATE_EMERGENCY         4
#define STATE_ERROR             5

/**
 * @brief PID控制器结构
 */
typedef struct {
    float kp;           // 比例增益
    float ki;           // 积分增益
    float kd;           // 微分增益
    float integral;     // 积分累计
    float prev_error;   // 上次误差
    float output_min;   // 输出下限
    float output_max;   // 输出上限
    float deadband;     // 死区
} PIDController;

/**
 * @brief 系统配置结构
 */
typedef struct {
    // 网络配置
    char wifi_ssid[32];
    char wifi_password[64];
    char remote_ip[16];
    uint16_t remote_port;
    
    // 控制参数
    float follow_distance;
    float max_linear_speed;
    float max_angular_speed;
    
    // PID参数
    float linear_kp, linear_ki, linear_kd;
    float angular_kp, angular_ki, angular_kd;
    
    // 安全参数
    float emergency_distance;
    float warning_distance;
    
} SystemConfig;

/* ==================== 工具函数宏 ==================== */

// 角度弧度转换
#define DEG_TO_RAD(deg)     ((deg) * 0.017453292519943295f)
#define RAD_TO_DEG(rad)     ((rad) * 57.295779513082323f)

// 限制范围
#define CLAMP(x, min, max)  (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

// 符号函数
#define SIGN(x)             (((x) > 0) ? 1 : (((x) < 0) ? -1 : 0))

// 最小/最大值
#define MIN(a, b)           (((a) < (b)) ? (a) : (b))
#define MAX(a, b)           (((a) > (b)) ? (a) : (b))

// 线性插值
#define LERP(a, b, t)       ((a) + (t) * ((b) - (a)))

// 计算校验和
static inline uint16_t calculate_checksum(const uint8_t* data, uint16_t length) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

// 获取系统时间戳(ms) - 需要平台实现
uint32_t get_timestamp_ms(void);

// 平方根近似（快速计算）
static inline float fast_sqrt(float x) {
    if (x <= 0) return 0;
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    return 1.0f / x;
}

// 反正切近似（快速计算）
static inline float fast_atan2(float y, float x) {
    return atan2f(y, x);
}

#ifdef __cplusplus
}
#endif

#endif /* COMMON_TYPES_H */
