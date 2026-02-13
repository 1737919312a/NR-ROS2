# ROS 2 云边端协同机器人系统

## 项目概述

本项目实现了一个完整的ROS 2云边端协同机器人系统，用于智能网联车创新设计比赛。系统采用分层架构设计，包含MCU层（ESP32-S3）、边缘端（Raspberry Pi 4B）和云端三个层级，实现了多车编队协同控制、车道检测与保持、AprilTag定位跟随、YOLO目标检测等核心功能。

## 目录结构

```
ros2-robot-system/
├── include/                    # 公共头文件
│   ├── common_types.h          # 公共类型定义
│   ├── pid_controller.h        # PID控制器接口
│   ├── pid_controller.c        # PID控制器实现
│   ├── ld06_lidar.h            # LD06激光雷达接口
│   └── ld06_lidar.c            # LD06激光雷达实现
├── mcu/                        # MCU层代码（ESP32-S3）
│   ├── host_mcu/               # 主机MCU
│   │   └── host_mcu_main.c     # 传感器融合+电机控制
│   └── slave_mcu/              # 从机MCU
│       └── slave_mcu_main.c    # 本地避障+跟随执行
├── edge/                       # 边缘端代码（RPi 4B）
│   ├── microros_bridge/        # Micro-ROS桥接节点（核心）
│   │   └── microros_bridge_node.cpp
│   ├── lane_detection/         # 车道检测节点
│   │   └── lane_detection_node.cpp
│   └── apriltag_localization/  # AprilTag定位节点
│       └── apriltag_node.cpp
├── cloud/                      # 云端代码
│   ├── yolo_detection/         # YOLO目标检测
│   │   └── yolo_detection_node.cpp
│   └── decision_comm/          # 决策通信模块
│       └── decision_comm.cpp
└── docs/                       # 文档
    └── README.md
```

---

## 一、MCU层（ESP32-S3）

### 1.1 主机MCU (host_mcu_main.c)

#### 功能说明
主机MCU作为主车的底层控制器，负责：
1. **传感器融合**：同时读取LD06激光雷达和毫米波雷达数据
2. **电机PID控制**：接收RPi的`/cmd_vel`指令，执行闭环速度控制
3. **Micro-ROS通信**：通过USB串口连接RPi Micro-ROS Agent
4. **状态监控**：上报电池电压、里程计、障碍物状态

#### 关键代码解析

```c
// 传感器融合避障检测
ObstacleStatus ld06_analyze_obstacles(const LidarScan* scan,
                                       const ObstacleConfig* config) {
    // 分析前方、左侧、右侧障碍物
    // 返回紧急停车标志和安全速度建议
}

// 速度控制主循环
void control_task(void* arg) {
    while (1) {
        // 1. 获取编码器增量
        // 2. 更新里程计
        // 3. 检查cmd_vel超时
        // 4. 执行避障判断
        // 5. 更新PID控制器
        // 6. 输出PWM到电机
    }
}
```

#### 注意事项

| 问题 | 说明 | 解决方案 |
|------|------|----------|
| **串口冲突** | USB和激光雷达同时使用UART | LD06使用UART1，Micro-ROS使用UART0(USB) |
| **内存限制** | ESP32-S3内存有限 | 使用固定大小缓冲区，避免动态分配 |
| **中断处理** | 编码器中断频繁 | 中断函数尽可能简短，只做计数 |
| **看门狗** | 通信中断可能导致失控 | 实现500ms超时急停机制 |
| **PID抖动** | 低速时电机抖动 | 设置速度死区，使用平滑滤波 |

#### 编译配置

```cmake
# CMakeLists.txt (ESP-IDF)
idf_component_register(SRCS "host_mcu_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos nvs_flash)

# sdkconfig
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_TASK_WDT_EN=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=5
```

---

### 1.2 从机MCU (slave_mcu_main.c)

#### 功能说明
从机MCU作为跟随车的底层控制器，核心功能是**本地激光避障**：
1. **独立避障**：前方0.3m强制停车，优先级最高
2. **跟随执行**：接收主机的跟随指令
3. **WiFi通信**：通过5G CPE连接RPi

#### 安全设计原则

```
┌─────────────────────────────────────────┐
│           优先级层级结构                │
├─────────────────────────────────────────┤
│  1. 本地激光避障（最高优先级）          │
│     ↓ 检测到障碍 → 立即停车            │
│                                        │
│  2. 指令超时保护                        │
│     ↓ 200ms无指令 → 减速停车           │
│                                        │
│  3. 远程跟随指令                        │
│     ↓ 正常执行                         │
└─────────────────────────────────────────┘
```

#### 关键代码

```c
// 本地避障检测（100Hz高频执行）
static bool local_obstacle_check(void) {
    ObstacleConfig config = {
        .emergency_distance = 0.30f,  // 紧急停车距离
        .warning_distance = 0.50f,    // 警告距离
        .front_fov = 90.0f,           // 前方90度检测
    };
    
    obstacle_status = ld06_analyze_obstacles(&latest_scan, &config);
    
    if (obstacle_status.min_distance <= EMERGENCY_DISTANCE) {
        return true;  // 需要紧急停车
    }
    return false;
}
```

#### 注意事项

- **避障响应时间**：必须小于100ms，确保在紧急情况下能及时停车
- **WiFi稳定性**：5G网络抖动时，本地决策不依赖网络
- **LED指示**：红灯表示紧急停车，闪烁表示前方有障碍

---

## 二、边缘端（Raspberry Pi 4B）

### 2.0 Micro-ROS桥接节点 (microros_bridge_node.cpp) ⭐核心模块

#### 功能说明
Micro-ROS桥接节点是边缘端的**核心通信枢纽**，负责MCU与ROS 2系统之间的数据转发：
1. **串口通信**：通过USB连接主机MCU (ESP32-S3)
2. **UDP通信**：通过WiFi连接从机MCU
3. **数据发布**：将MCU数据转换为ROS 2消息发布
4. **指令转发**：将ROS 2指令下发到MCU

#### 通信架构

```
                    ┌─────────────────────────────────────┐
                    │      Raspberry Pi 4B (边缘端)       │
                    │                                     │
   [主机MCU] ◄────►│  ┌─────────────────────────────┐   │
   USB Serial       │  │   Micro-ROS Bridge Node     │   │
   (921600 baud)    │  │                             │   │
                    │  │  • LaserScan 发布           │◄──┼──► /scan
   [从机MCU] ◄────►│  │  • Odometry 发布            │◄──┼──► /odom
   WiFi UDP         │  │  • cmd_vel 订阅             │◄──┼──► /cmd_vel
   (Port 8889)      │  │  • follow_cmd 订阅          │◄──┼──► /follow_cmd
                    │  └─────────────────────────────┘   │
                    └─────────────────────────────────────┘
```

#### 关键话题

| 话题名称 | 方向 | 消息类型 | 说明 |
|----------|------|----------|------|
| `/scan` | 发布 | LaserScan | 激光扫描数据 |
| `/odom` | 发布 | Odometry | 里程计数据 |
| `/obstacle_status` | 发布 | Float32 | 障碍物距离 |
| `/cmd_vel` | 订阅 | Twist | 速度指令 |
| `/follow_cmd` | 订阅 | Twist | 跟随指令 |
| `/bridge_connected` | 发布 | Bool | 连接状态 |
| `/bridge_status` | 发布 | String | 详细状态 |

#### 数据包格式

```c
// 串口数据包 (主机MCU)
struct SerialPacket {
    uint8_t header[2];      // 0xAA 0x55
    uint8_t type;           // 包类型
    // 数据载荷...
    uint16_t checksum;      // 校验和
};

// UDP数据包 (从机MCU)
struct UDPPacket {
    uint8_t header[2];      // 0xAA 0x56
    uint8_t type;           // 包类型
    // 数据载荷...
    uint16_t checksum;
};
```

#### 注意事项

| 问题 | 说明 | 解决方案 |
|------|------|----------|
| **串口权限** | 需要权限访问/dev/ttyUSB0 | `sudo usermod -a -G dialout $USER` |
| **设备变化** | USB重新枚举导致设备名变化 | 使用udev规则创建固定符号链接 |
| **超时检测** | MCU通信中断 | 1秒无数据标记为断开 |
| **数据校验** | 传输错误 | 使用校验和验证 |

#### 启动示例

```bash
# 基本启动
ros2 run ros2_robot_system microros_bridge_node

# 带参数启动
ros2 run ros2_robot_system microros_bridge_node \
    --ros-args \
    -p serial_device:=/dev/ttyUSB0 \
    -p serial_baud:=921600 \
    -p udp_port_slave:=8889

# 通过launch文件启动
ros2 launch ros2_robot_system edge_nodes.launch.py \
    microros_bridge:=true \
    serial_device:=/dev/ttyUSB0
```

---

### 2.1 车道检测节点 (lane_detection_node.cpp)

#### 功能说明
基于OpenCV的实时车道线检测与保持控制：
1. **车道线检测**：HSV颜色空间 + 边缘检测 + 霍夫直线
2. **偏移计算**：计算车辆相对车道中心的横向偏移
3. **航向误差**：计算车辆航向与车道方向的偏差
4. **速度发布**：输出`/cmd_vel_lane`实现车道保持

#### 算法流程

```
输入图像 → ROI裁剪 → HSV转换 → 颜色分割(白/黄)
    ↓
高斯模糊 → Canny边缘检测 → 霍夫直线变换
    ↓
左右车道线分离 → 直线拟合 → 计算偏移和航向
    ↓
PID控制 → 输出速度指令
```

#### 参数调优指南

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|----------|
| `roi_top` | 300 | ROI顶部边界 | 根据摄像头安装角度调整 |
| `kp_angular` | 0.8 | 角速度比例增益 | 增大则转向更灵敏，但可能震荡 |
| `kp_linear` | 0.3 | 线速度比例增益 | 根据车辆动力学调整 |
| `white_low/high` | (0,0,200)/(180,30,255) | 白色阈值 | 根据光照条件调整 |

#### 性能优化

```cpp
// 使用GPU加速（RPi 4B支持）
cv::UMat gpu_img, gpu_hsv;
frame.copyTo(gpu_img);
cv::cvtColor(gpu_img, gpu_hsv, cv::COLOR_BGR2HSV);

// ROI裁剪减少计算量
cv::Rect roi(0, params_.roi_top, frame.cols, 
             params_.roi_bottom - params_.roi_top);
cv::Mat roi_img = frame(roi);
```

---

### 2.2 AprilTag定位节点 (apriltag_node.cpp)

#### 功能说明
检测从机上的AprilTag标签，计算相对位姿并发布跟随指令：
1. **AprilTag检测**：使用36h11标签族
2. **位姿估计**：计算3D位置和朝向角
3. **跟随控制**：输出`/follow_cmd`给从机

#### 相机标定

必须先进行相机标定获取内参：

```bash
# 使用ROS 2相机标定工具
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 \
    --camera /camera/rear --namespace /camera/rear
```

标定参数更新到代码中：
```cpp
double fx = 309.0;  // 焦距x
double fy = 309.0;  // 焦距y
double cx = 320.0;  // 光心x
double cy = 240.0;  // 光心y
```

#### 注意事项

- **标签尺寸**：`tag_size`参数必须准确测量（单位：米）
- **检测距离**：640x480分辨率下，有效检测距离约2米
- **多标签**：支持多标签检测，使用第一个检测到的标签

---

## 三、云端

### 3.1 YOLO检测节点 (yolo_detection_node.cpp)

#### 功能说明
云端运行YOLO进行语义认知：
1. **目标检测**：识别红绿灯、行人、车辆等
2. **决策下发**：根据检测结果输出高层决策

#### 检测类别

| 类别ID | 名称 | 触发决策 |
|--------|------|----------|
| 100 | 红灯 | TRAFFIC_LIGHT_WAIT |
| 101 | 绿灯 | CONTINUE |
| 0 | 行人 | PEDESTRIAN_AVOID |
| 2,5,7 | 车辆 | SLOW_DOWN |

#### 部署要求

```bash
# 安装依赖
pip install tensorrt onnxruntime-gpu

# 转换模型为TensorRT引擎
yolo export model=yolov8n.pt format=engine device=0
```

---

### 3.2 决策通信模块 (decision_comm.cpp)

#### 功能说明
实现可靠的云边端通信：
1. **UDP传输**：低延迟通信
2. **确认重传**：保证可靠性
3. **校验机制**：检测传输错误

#### 通信协议

```
决策包格式 (24字节):
┌─────────┬─────────┬──────────┬──────────┬─────────┬──────────┬──────────┬─────────┐
│ Header  │ Command │ Priority │ Param1   │ Param2  │ Sequence │Timestamp │Checksum │
│ (2B)    │ (1B)    │ (1B)     │ (4B)     │ (4B)    │ (4B)     │ (4B)     │ (2B)    │
└─────────┴─────────┴──────────┴──────────┴─────────┴──────────┴──────────┴─────────┘

确认包格式 (12字节):
┌─────────┬─────────────┬──────────┬──────────┐
│ Header  │ Ack_Sequence│ Timestamp│ Checksum │
│ (2B)    │ (4B)        │ (4B)     │ (2B)     │
└─────────┴─────────────┴──────────┴──────────┘
```

---

## 四、编译与部署

### 4.1 MCU编译（ESP-IDF）

```bash
# 安装ESP-IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh

# 编译主机MCU
cd mcu/host_mcu
idf.py build
idf.py flash

# 编译从机MCU
cd mcu/slave_mcu
idf.py build
idf.py flash
```

### 4.2 边缘端编译（ROS 2 Humble）

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s /path/to/ros2-robot-system/edge .

# 编译
cd ~/ros2_ws
colcon build --symlink-install

# 运行车道检测
ros2 run lane_detection lane_detection_node

# 运行AprilTag定位
ros2 run apriltag_localization apriltag_node
```

### 4.3 云端部署

```bash
# 安装依赖
pip install ultralytics tensorrt

# 运行YOLO检测
ros2 run yolo_detection yolo_detection_node
```

---

## 五、安全机制

### 5.1 多级看门狗

```
┌────────────────────────────────────────────────────┐
│                   看门狗层级                        │
├────────────────────────────────────────────────────┤
│ Level 1: 硬件看门狗（ESP32内部）                   │
│         超时5秒 → 系统复位                        │
│                                                    │
│ Level 2: 软件看门狗                                │
│         监控通信链路，超时500ms → 急停            │
│                                                    │
│ Level 3: 指令超时                                  │
│         200ms无新指令 → 减速停车                  │
└────────────────────────────────────────────────────┘
```

### 5.2 故障降级策略

| 故障类型 | 系统响应 |
|----------|----------|
| 云端断连 | 边缘端继续本地控制 |
| 激光雷达故障 | 限制最大速度为0.2m/s |
| 编码器故障 | 停车并上报错误 |
| WiFi断连 | 从机执行安全停车 |

---

## 六、已知问题与解决

### 6.1 NR-V2X替代方案

**问题**：比赛要求NR-V2X通信，但方案使用5G+WiFi替代。

**解决方案**：
1. 确认NR-V2X是否为强制要求
2. 如需实现，采购NR-V2X模组（如移远AG15）
3. 当前方案通过优化UDP重传机制补偿可靠性

### 6.2 小尺寸场地适配

**问题**：3m×4.5m场地对编队控制精度要求高。

**解决方案**：
1. 减小跟随距离至0.4m
2. 降低最大速度至0.3m/s
3. 提高控制频率至100Hz

---

## 七、版本历史

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| v1.0 | 2026-02-13 | 初始版本，完成所有核心模块 |

---

## 八、联系方式

如有问题，请联系开发团队。
