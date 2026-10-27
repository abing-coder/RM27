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
  )
endif()

# 达妙 DM-USB2FDCAN SDK（can_interface: "damiao" 时使用）。
# 默认必须编译并链接达妙 SDK（上车实测不能漏），找不到匹配架构的库会直接报错而不是静默跳过。
# 仅在本机用视频做纯视觉调试（无 CAN）时，显式配置 -DVIDEO_ONLY=ON 才跳过达妙 SDK：
#   cmake -B build -DVIDEO_ONLY=ON
# 仓库自带的 lib/linux/libdm_device.so 仅为 x86-64；arm64 机器请把对应架构的 .so 放到 lib/linux_arm64/。
set(DAMIAO_ENABLED OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(DAMIAO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/damiao/lib/linux)
  else()
    set(DAMIAO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/damiao/lib/linux_arm64)
  endif()
  if(VIDEO_ONLY)
    message(STATUS "VIDEO_ONLY=ON: skipping Damiao CAN SDK (can_interface: \"damiao\" unavailable, video debugging only).")
  elseif(EXISTS ${DAMIAO_LIB_DIR}/libdm_device.so)
    set(DAMIAO_ENABLED ON)
    list(APPEND IO_SOURCES damiao_can.cpp)
    message(STATUS "Damiao CAN SDK found: ${DAMIAO_LIB_DIR}")
  else()
    message(FATAL_ERROR
      "Damiao CAN SDK not found for ${CMAKE_SYSTEM_PROCESSOR}: ${DAMIAO_LIB_DIR}/libdm_device.so\n"
      "  - 上车实测：请把对应架构的 libdm_device.so 放到该目录后重新 cmake\n"
      "  - 仅本机视频调试（无 CAN）：cmake -B build -DVIDEO_ONLY=ON")
  endif()
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

  target_link_libraries(io PUBLIC MvCameraControl MVSDK usb-1.0 yaml-cpp serial)

  if(DAMIAO_ENABLED)
    target_compile_definitions(io PUBLIC SP_HAS_DAMIAO_CAN)
    target_link_directories(io PUBLIC ${DAMIAO_LIB_DIR})
    target_link_libraries(io PUBLIC dm_device)
  endif()
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
