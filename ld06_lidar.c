/**
 * @file ld06_lidar.c
 * @brief LD06激光雷达驱动实现 - ESP32-S3专用
 */

#include "ld06_lidar.h"
#include <string.h>
#include <math.h>

// ESP32专用头文件（ESP-IDF框架）
#ifdef ESP_PLATFORM
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
static const char* TAG = "LD06";
#endif

/* ==================== CRC8查找表 ==================== */

static const uint8_t crc8_table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
};

/* ==================== CRC计算 ==================== */

uint8_t ld06_crc8(const uint8_t* data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

bool ld06_verify_packet(const LD06Packet* packet) {
    // 检查头字节
    if (packet->header != LD06_HEADER_BYTE) {
        return false;
    }
    
    // 计算CRC并验证
    uint8_t crc = ld06_crc8((const uint8_t*)packet, LD06_PACKET_SIZE - 1);
    return (crc == packet->crc8);
}

/* ==================== 初始化与配置 ==================== */

int ld06_init(LD06Driver* driver, int uart_num, int tx_pin, int rx_pin) {
    memset(driver, 0, sizeof(LD06Driver));
    
    driver->uart_num = uart_num;
    driver->tx_pin = tx_pin;
    driver->rx_pin = rx_pin;
    driver->baud_rate = 230400;  // LD06默认波特率
    
#ifdef ESP_PLATFORM
    // ESP32 UART配置
    uart_config_t uart_config = {
        .baud_rate = driver->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %d", ret);
        return -1;
    }
    
    ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %d", ret);
        return -2;
    }
    
    // 安装UART驱动，缓冲区大小为2个数据包
    ret = uart_driver_install(uart_num, LD06_PACKET_SIZE * 4, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %d", ret);
        return -3;
    }
    
    ESP_LOGI(TAG, "LD06 initialized on UART%d", uart_num);
#endif
    
    driver->initialized = true;
    return 0;
}

int ld06_start(LD06Driver* driver) {
    if (!driver->initialized) {
        return -1;
    }
    
    driver->running = true;
    driver->scan_index = 0;
    driver->scan_ready = false;
    
    // LD06默认持续输出，无需发送启动命令
    return 0;
}

void ld06_stop(LD06Driver* driver) {
    driver->running = false;
}

void ld06_deinit(LD06Driver* driver) {
    driver->running = false;
    driver->initialized = false;
    
#ifdef ESP_PLATFORM
    uart_driver_delete(driver->uart_num);
#endif
}

/* ==================== 数据读取与解析 ==================== */

/**
 * @brief 解析单个数据点
 */
static void parse_point(LidarPoint* point, const LD06RawPoint* raw, float angle) {
    // 距离转换：原始值单位mm，转换为m
    point->distance = raw->distance / 1000.0f;
    point->intensity = raw->intensity;
    point->angle = angle;
    
    // 有效性检查
    point->valid = (point->distance >= LD06_MIN_DISTANCE && 
                    point->distance <= LD06_MAX_DISTANCE &&
                    raw->intensity > 0);
}

bool ld06_process(LD06Driver* driver) {
    if (!driver->running) {
        return false;
    }
    
#ifdef ESP_PLATFORM
    // 读取串口数据
    int len = uart_read_bytes(driver->uart_num, 
                               driver->rx_buffer + driver->rx_index,
                               LD06_PACKET_SIZE,
                               1);  // 1 tick timeout
    
    if (len <= 0) {
        return false;
    }
    
    driver->rx_index += len;
    
    // 检查是否收到完整数据包
    while (driver->rx_index >= LD06_PACKET_SIZE) {
        // 查找包头
        int header_pos = -1;
        for (int i = 0; i <= driver->rx_index - LD06_PACKET_SIZE; i++) {
            if (driver->rx_buffer[i] == LD06_HEADER_BYTE) {
                header_pos = i;
                break;
            }
        }
        
        if (header_pos < 0) {
            // 没找到包头，丢弃所有数据
            driver->rx_index = 0;
            return false;
        }
        
        // 移动数据到缓冲区开始
        if (header_pos > 0) {
            memmove(driver->rx_buffer, 
                    driver->rx_buffer + header_pos,
                    driver->rx_index - header_pos);
            driver->rx_index -= header_pos;
        }
        
        // 解析数据包
        LD06Packet* packet = (LD06Packet*)driver->rx_buffer;
        
        if (!ld06_verify_packet(packet)) {
            driver->packets_error++;
            // 丢弃一个字节继续查找
            memmove(driver->rx_buffer, 
                    driver->rx_buffer + 1,
                    driver->rx_index - 1);
            driver->rx_index--;
            continue;
        }
        
        driver->packets_received++;
        
        // 计算角度步进
        float start_angle = packet->start_angle / 100.0f;  // 转换为度
        float end_angle = packet->end_angle / 100.0f;
        float angle_step = (end_angle - start_angle) / (LD06_POINTS_PER_PACKET - 1);
        
        // 处理跨越360度的情况
        if (angle_step < 0) {
            angle_step += 360.0f / (LD06_POINTS_PER_PACKET - 1);
        }
        
        // 解析12个数据点
        for (int i = 0; i < LD06_POINTS_PER_PACKET; i++) {
            float angle = start_angle + angle_step * i;
            if (angle >= 360.0f) angle -= 360.0f;
            
            uint16_t idx = driver->scan_index;
            if (idx < LD06_POINTS_PER_SCAN) {
                parse_point(&driver->current_scan.points[idx],
                           &packet->points[i], angle);
                driver->scan_index++;
            }
        }
        
        // 检查是否完成一圈扫描（角度回到起点附近）
        if (end_angle < start_angle || end_angle < 10.0f) {
            // 完成一圈
            driver->current_scan.point_count = driver->scan_index;
            driver->current_scan.timestamp = get_timestamp_ms();
            driver->current_scan.scan_time = 1.0f / LD06_SCAN_FREQ;
            
            // 复制到输出缓冲
            memcpy(&driver->complete_scan, &driver->current_scan, sizeof(LidarScan));
            
            driver->scan_index = 0;
            driver->scan_ready = true;
        }
        
        // 移除已处理的数据包
        driver->rx_index -= LD06_PACKET_SIZE;
        if (driver->rx_index > 0) {
            memmove(driver->rx_buffer,
                    driver->rx_buffer + LD06_PACKET_SIZE,
                    driver->rx_index);
        }
        
        return driver->scan_ready;
    }
#endif
    
    return false;
}

bool ld06_get_scan(LD06Driver* driver, LidarScan* scan) {
    if (!driver->scan_ready) {
        return false;
    }
    
    memcpy(scan, &driver->complete_scan, sizeof(LidarScan));
    driver->scan_ready = false;
    return true;
}

bool ld06_scan_ready(LD06Driver* driver) {
    return driver->scan_ready;
}

void ld06_clear_scan_ready(LD06Driver* driver) {
    driver->scan_ready = false;
}

/* ==================== 点云处理 ==================== */

void ld06_filter_points(LidarScan* scan, float min_dist, float max_dist) {
    for (uint16_t i = 0; i < scan->point_count; i++) {
        LidarPoint* p = &scan->points[i];
        if (p->distance < min_dist || p->distance > max_dist) {
            p->valid = false;
        }
    }
}

bool ld06_get_nearest_in_range(const LidarScan* scan,
                                float angle_min, float angle_max,
                                float* distance, float* angle) {
    float min_dist = LD06_MAX_DISTANCE + 1.0f;
    float min_angle = 0.0f;
    bool found = false;
    
    for (uint16_t i = 0; i < scan->point_count; i++) {
        const LidarPoint* p = &scan->points[i];
        if (!p->valid) continue;
        
        // 检查角度范围
        float a = p->angle;
        if (a >= angle_min && a <= angle_max) {
            if (p->distance < min_dist) {
                min_dist = p->distance;
                min_angle = a;
                found = true;
            }
        }
    }
    
    if (found) {
        *distance = min_dist;
        *angle = min_angle;
    }
    
    return found;
}

ObstacleInfo ld06_get_front_obstacle(const LidarScan* scan, 
                                      float fov, float max_distance) {
    ObstacleInfo obs = {0};
    obs.active = false;
    obs.distance = max_distance;
    
    float half_fov = fov / 2.0f;
    float angle_min = 360.0f - half_fov;  // 处理跨0度的情况
    float angle_max = half_fov;
    
    for (uint16_t i = 0; i < scan->point_count; i++) {
        const LidarPoint* p = &scan->points[i];
        if (!p->valid) continue;
        
        // 检查是否在前方扇区内
        bool in_fov = (p->angle <= angle_max) || (p->angle >= angle_min);
        
        if (in_fov && p->distance < max_distance) {
            if (p->distance < obs.distance) {
                obs.distance = p->distance;
                obs.angle = p->angle;
                obs.source = 0;  // 激光雷达
                obs.active = true;
            }
        }
    }
    
    return obs;
}

void ld06_sector_analysis(const LidarScan* scan, uint8_t sector_count,
                          float* distances, float* angles) {
    float sector_size = 360.0f / sector_count;
    
    // 初始化
    for (int i = 0; i < sector_count; i++) {
        distances[i] = LD06_MAX_DISTANCE + 1.0f;
        angles[i] = 0.0f;
    }
    
    // 分析每个点
    for (uint16_t i = 0; i < scan->point_count; i++) {
        const LidarPoint* p = &scan->points[i];
        if (!p->valid) continue;
        
        int sector = (int)(p->angle / sector_size);
        if (sector >= sector_count) sector = sector_count - 1;
        
        if (p->distance < distances[sector]) {
            distances[sector] = p->distance;
            angles[sector] = p->angle;
        }
    }
}

/* ==================== 避障检测 ==================== */

ObstacleStatus ld06_analyze_obstacles(const LidarScan* scan,
                                       const ObstacleConfig* config) {
    ObstacleStatus status = {0};
    status.timestamp = get_timestamp_ms();
    
    float half_front = config->front_fov / 2.0f;
    float half_side = config->side_fov / 2.0f;
    
    // 前方区域检测
    float dist, angle;
    float front_min = LD06_MAX_DISTANCE + 1.0f;
    
    // 正前方 (0度附近)
    for (uint16_t i = 0; i < scan->point_count; i++) {
        const LidarPoint* p = &scan->points[i];
        if (!p->valid) continue;
        
        float a = p->angle;
        
        // 前方 (考虑跨0度)
        bool in_front = (a <= half_front) || (a >= 360.0f - half_front);
        // 左前方
        bool in_left = (a >= half_front) && (a <= half_front + config->side_fov);
        // 右前方
        bool in_right = (a >= 360.0f - half_front - config->side_fov) && 
                        (a < 360.0f - half_front);
        
        if (in_front || in_left || in_right) {
            if (p->distance < front_min) {
                front_min = p->distance;
            }
            
            if (p->distance <= config->emergency_distance) {
                status.emergency_stop = true;
            }
            
            if (in_front && p->distance <= config->warning_distance) {
                status.obstacle_front = true;
            }
            if (in_left && p->distance <= config->warning_distance) {
                status.obstacle_left = true;
            }
            if (in_right && p->distance <= config->warning_distance) {
                status.obstacle_right = true;
            }
        }
    }
    
    status.min_distance = front_min;
    
    // 计算安全速度
    if (status.emergency_stop) {
        status.safe_speed = 0.0f;
    } else if (front_min < config->warning_distance) {
        // 线性降速
        float ratio = (front_min - config->emergency_distance) /
                      (config->warning_distance - config->emergency_distance);
        status.safe_speed = config->max_safe_speed * ratio;
    } else {
        status.safe_speed = config->max_safe_speed;
    }
    
    return status;
}

float ld06_compute_safe_speed(const LidarScan* scan,
                               float desired_speed,
                               const ObstacleConfig* config) {
    ObstacleStatus status = ld06_analyze_obstacles(scan, config);
    return MIN(desired_speed, status.safe_speed);
}

/* ==================== 坐标转换 ==================== */

void ld06_polar_to_cartesian(float angle, float distance, float* x, float* y) {
    float rad = DEG_TO_RAD(angle);
    *x = distance * cosf(rad);
    *y = distance * sinf(rad);
}

void ld06_scan_to_cartesian(const LidarScan* scan,
                             float* x_array, float* y_array, uint16_t* count) {
    uint16_t valid_count = 0;
    
    for (uint16_t i = 0; i < scan->point_count; i++) {
        const LidarPoint* p = &scan->points[i];
        if (p->valid) {
            ld06_polar_to_cartesian(p->angle, p->distance,
                                    &x_array[valid_count],
                                    &y_array[valid_count]);
            valid_count++;
        }
    }
    
    *count = valid_count;
}
