/**
 * @file ld06_lidar.h
 * @brief LD06激光雷达驱动程序 - ESP32-S3专用
 * 
 * LD06是一款低成本360度激光雷达，通过UART接口通信。
 * 本驱动实现了数据解析、点云处理和避障检测功能。
 * 
 * 技术规格：
 * - 扫描频率：12Hz
 * - 角度分辨率：1度
 * - 测距范围：0.05m - 12m
 * - 数据包长度：46字节
 * - 波特率：230400
 */

#ifndef LD06_LIDAR_H
#define LD06_LIDAR_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== LD06数据包结构 ==================== */

// LD06数据包头
#define LD06_HEADER_BYTE        0x54
#define LD06_PACKET_SIZE        46
#define LD06_POINTS_PER_PACKET  12

/**
 * @brief LD06数据点结构（原始格式）
 */
typedef struct __attribute__((packed)) {
    uint16_t distance;      // 距离(mm)
    uint8_t intensity;      // 强度
} LD06RawPoint;

/**
 * @brief LD06数据包头结构
 */
typedef struct __attribute__((packed)) {
    uint8_t header;         // 头字节 0x54
    uint8_t ver_len;        // 版本和长度
    uint16_t speed;         // 转速(度/秒)
    uint16_t start_angle;   // 起始角度(0.01度)
    LD06RawPoint points[12]; // 12个数据点
    uint16_t end_angle;     // 结束角度(0.01度)
    uint8_t timestamp;      // 时间戳
    uint8_t crc8;           // CRC校验
} LD06Packet;

/**
 * @brief LD06激光雷达驱动结构
 */
typedef struct {
    // 串口配置
    int uart_num;           // UART端口号
    int tx_pin;             // TX引脚
    int rx_pin;             // RX引脚
    int baud_rate;          // 波特率
    
    // 数据缓冲
    uint8_t rx_buffer[LD06_PACKET_SIZE * 2];
    uint16_t rx_index;
    
    // 扫描数据
    LidarScan current_scan;
    LidarScan complete_scan;
    uint16_t scan_index;
    bool scan_ready;
    
    // 统计信息
    uint32_t packets_received;
    uint32_t packets_error;
    uint32_t last_timestamp;
    
    // 状态
    bool initialized;
    bool running;
    
} LD06Driver;

/* ==================== 初始化与配置 ==================== */

/**
 * @brief 初始化LD06驱动
 * @param driver 驱动结构指针
 * @param uart_num UART端口号
 * @param tx_pin TX引脚号
 * @param rx_pin RX引脚号
 * @return 0成功，负数失败
 */
int ld06_init(LD06Driver* driver, int uart_num, int tx_pin, int rx_pin);

/**
 * @brief 启动LD06扫描
 * @param driver 驱动结构指针
 * @return 0成功，负数失败
 */
int ld06_start(LD06Driver* driver);

/**
 * @brief 停止LD06扫描
 * @param driver 驱动结构指针
 */
void ld06_stop(LD06Driver* driver);

/**
 * @brief 释放LD06资源
 * @param driver 驱动结构指针
 */
void ld06_deinit(LD06Driver* driver);

/* ==================== 数据读取与解析 ==================== */

/**
 * @brief 处理串口数据（在主循环中调用）
 * @param driver 驱动结构指针
 * @return true如果获得完整扫描
 */
bool ld06_process(LD06Driver* driver);

/**
 * @brief 获取完整扫描数据
 * @param driver 驱动结构指针
 * @param scan 输出扫描数据
 * @return true如果获取成功
 */
bool ld06_get_scan(LD06Driver* driver, LidarScan* scan);

/**
 * @brief 检查是否有新的扫描数据
 * @param driver 驱动结构指针
 * @return true如果有新数据
 */
bool ld06_scan_ready(LD06Driver* driver);

/**
 * @brief 清除扫描就绪标志
 * @param driver 驱动结构指针
 */
void ld06_clear_scan_ready(LD06Driver* driver);

/* ==================== CRC校验 ==================== */

/**
 * @brief 计算CRC8校验
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC8值
 */
uint8_t ld06_crc8(const uint8_t* data, uint16_t len);

/**
 * @brief 验证数据包CRC
 * @param packet 数据包指针
 * @return true如果校验通过
 */
bool ld06_verify_packet(const LD06Packet* packet);

/* ==================== 点云处理 ==================== */

/**
 * @brief 过滤无效点
 * @param scan 扫描数据指针
 * @param min_dist 最小有效距离(m)
 * @param max_dist 最大有效距离(m)
 */
void ld06_filter_points(LidarScan* scan, float min_dist, float max_dist);

/**
 * @brief 获取指定角度范围内的最近点
 * @param scan 扫描数据指针
 * @param angle_min 最小角度(deg)
 * @param angle_max 最大角度(deg)
 * @param distance 输出最近距离
 * @param angle 输出对应角度
 * @return true如果找到有效点
 */
bool ld06_get_nearest_in_range(const LidarScan* scan, 
                                float angle_min, float angle_max,
                                float* distance, float* angle);

/**
 * @brief 获取前方障碍物信息
 * @param scan 扫描数据指针
 * @param fov 视场角(deg)，对称分布
 * @param max_distance 最大检测距离(m)
 * @return 障碍物信息
 */
ObstacleInfo ld06_get_front_obstacle(const LidarScan* scan, 
                                      float fov, float max_distance);

/**
 * @brief 扇区分析
 * @param scan 扫描数据指针
 * @param sector_count 扇区数量
 * @param distances 输出各扇区最近距离数组
 * @param angles 输出各扇区最近点角度数组
 */
void ld06_sector_analysis(const LidarScan* scan, uint8_t sector_count,
                          float* distances, float* angles);

/* ==================== 避障检测 ==================== */

/**
 * @brief 避障配置
 */
typedef struct {
    float emergency_distance;   // 紧急停车距离(m)
    float warning_distance;     // 警告距离(m)
    float front_fov;            // 前方视场角(deg)
    float side_fov;             // 侧方视场角(deg)
    float max_safe_speed;       // 最大安全速度(m/s)
} ObstacleConfig;

/**
 * @brief 分析障碍物状态
 * @param scan 扫描数据指针
 * @param config 避障配置
 * @return 避障状态
 */
ObstacleStatus ld06_analyze_obstacles(const LidarScan* scan, 
                                       const ObstacleConfig* config);

/**
 * @brief 计算安全速度建议
 * @param scan 扫描数据指针
 * @param desired_speed 期望速度(m/s)
 * @param config 避障配置
 * @return 安全速度(m/s)
 */
float ld06_compute_safe_speed(const LidarScan* scan, 
                               float desired_speed,
                               const ObstacleConfig* config);

/* ==================== 坐标转换 ==================== */

/**
 * @brief 极坐标转笛卡尔坐标
 * @param angle 角度(deg)
 * @param distance 距离(m)
 * @param x 输出X坐标(m)
 * @param y 输出Y坐标(m)
 */
void ld06_polar_to_cartesian(float angle, float distance, float* x, float* y);

/**
 * @brief 批量转换点云到笛卡尔坐标
 * @param scan 扫描数据指针
 * @param x_array 输出X坐标数组
 * @param y_array 输出Y坐标数组
 * @param count 输出有效点数
 */
void ld06_scan_to_cartesian(const LidarScan* scan, 
                             float* x_array, float* y_array, uint16_t* count);

#ifdef __cplusplus
}
#endif

#endif /* LD06_LIDAR_H */
