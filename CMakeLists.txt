cmake_minimum_required(VERSION 3.8)
project(ros2_robot_system)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ========================================
# 车道检测节点
# ========================================
add_executable(lane_detection_node
    lane_detection_node.cpp
)

target_include_directories(lane_detection_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lane_detection_node
    ${OpenCV_LIBS}
)

ament_target_dependencies(lane_detection_node
    rclcpp
    sensor_msgs
    geometry_msgs
    cv_bridge
)

# ========================================
# AprilTag定位节点
# ========================================
add_executable(apriltag_node
    apriltag_node.cpp
)

target_include_directories(apriltag_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)

# AprilTag库（需要单独安装）
find_library(APRILTAG_LIBRARY apriltag)
if(APRILTAG_LIBRARY)
    target_link_libraries(apriltag_node
        ${OpenCV_LIBS}
        ${APRILTAG_LIBRARY}
    )
else()
    message(WARNING "AprilTag library not found. Please install: sudo apt install libapriltag-dev")
endif()

ament_target_dependencies(apriltag_node
    rclcpp
    sensor_msgs
    geometry_msgs
    cv_bridge
)

# ========================================
# YOLO检测节点（云端）
# ========================================
add_executable(yolo_detection_node
    yolo_detection_node.cpp
)

target_include_directories(yolo_detection_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(yolo_detection_node
    ${OpenCV_LIBS}
)

ament_target_dependencies(yolo_detection_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
)

# ========================================
# 决策通信节点（云端）
# ========================================
add_executable(decision_comm_node
    decision_comm.cpp
)

target_include_directories(decision_comm_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(decision_comm_node
    rclcpp
    std_msgs
    geometry_msgs
)

# ========================================
# 安装目标
# ========================================
install(TARGETS
    lane_detection_node
    apriltag_node
    yolo_detection_node
    decision_comm_node
    DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
    install(DIRECTORY
        include/
        DESTINATION include
    )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/docs)
    install(DIRECTORY
        docs/
        DESTINATION share/${PROJECT_NAME}
    )
endif()

# ========================================
# 测试（可选）
# ========================================
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    enable_testing()
    add_executable(pid_controller_test
        tests/pid_controller_test.c
        pid_controller.c
    )
    target_include_directories(pid_controller_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(pid_controller_test m)
    add_test(NAME pid_controller_test COMMAND pid_controller_test)
endif()

ament_package()
