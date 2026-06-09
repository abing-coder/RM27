cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(serial)

# 跨平台源文件（不依赖工业相机 SDK / SocketCAN / 达妙 CAN）
set(IO_SOURCES
    usbcamera/usbcamera.cpp
    video/video.cpp
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# 工业相机 SDK 与 CAN 仅在 Linux 上可用（SDK 为 Linux .so，SocketCAN 依赖 linux/can.h）。
# 非 Linux（如 macOS）下自动跳过，相机只能用 camera_name: "video" 跑视频调试。
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  list(APPEND IO_SOURCES
    hikrobot/hikrobot.cpp
    mindvision/mindvision.cpp
    damiao_can.cpp
  )
endif()

# 创建目标 io
add_library(io STATIC ${IO_SOURCES})

target_link_libraries(io PUBLIC Eigen3::Eigen)

# hikrobot / mindvision 头文件（SDK 头文件随仓库提供，编译期可用）
target_include_directories(io PUBLIC hikrobot/include mindvision/include)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # hikrobot / mindvision SDK 库目录（按架构选择）
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(io PUBLIC hikrobot/lib/amd64 mindvision/lib/amd64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_link_directories(io PUBLIC hikrobot/lib/arm64 mindvision/lib/arm64)
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
  endif()

  # 达妙 DM-USB2FDCAN SDK（can_interface: "damiao" 时使用）
  target_link_directories(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/damiao/lib/linux)
  target_link_libraries(io PUBLIC MvCameraControl MVSDK usb-1.0 dm_device yaml-cpp serial)
else()
  # 非 Linux：跳过工业相机 SDK 与 CAN，仅链接可跨平台的依赖
  target_link_libraries(io PUBLIC yaml-cpp serial)
endif()

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 设置 ROS2 依赖
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs)
endif()
