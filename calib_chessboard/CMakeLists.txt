cmake_minimum_required(VERSION 3.16)
project(chessboard_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Calibration does not need inference models, OpenVINO, ROS or the CAN SDK.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio calib3d)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
add_subdirectory("${REPO_ROOT}/io/serial" "${CMAKE_CURRENT_BINARY_DIR}/serial")
add_executable(calib_chessboard
  calib.cpp
  "${REPO_ROOT}/io/camera.cpp"
  "${REPO_ROOT}/io/video/video.cpp"
  "${REPO_ROOT}/io/dm_imu/dm_imu.cpp"
  "${REPO_ROOT}/tools/img_tools.cpp"
  "${REPO_ROOT}/tools/math_tools.cpp"
  "${REPO_ROOT}/tools/logger.cpp")
target_include_directories(calib_chessboard PRIVATE "${REPO_ROOT}" ${OpenCV_INCLUDE_DIRS})
target_compile_options(calib_chessboard PRIVATE
  -include "${REPO_ROOT}/tools/fmt_enum_compat.hpp")
target_link_libraries(calib_chessboard PRIVATE
  ${OpenCV_LIBS} Eigen3::Eigen fmt::fmt spdlog::spdlog yaml-cpp serial Threads::Threads)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(SDK_ARCH amd64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(SDK_ARCH arm64)
  else()
    message(FATAL_ERROR "Unsupported camera SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
  target_sources(calib_chessboard PRIVATE
    "${REPO_ROOT}/io/hikrobot/hikrobot.cpp"
    "${REPO_ROOT}/io/mindvision/mindvision.cpp")
  target_include_directories(calib_chessboard PRIVATE
    "${REPO_ROOT}/io/hikrobot/include" "${REPO_ROOT}/io/mindvision/include")
  target_link_libraries(calib_chessboard PRIVATE PkgConfig::LIBUSB
    "${REPO_ROOT}/io/hikrobot/lib/${SDK_ARCH}/libMvCameraControl.so"
    "${REPO_ROOT}/io/mindvision/lib/${SDK_ARCH}/libMVSDK.so")
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(calibration_session_test "${REPO_ROOT}/tests/calibration_session_test.cpp")
  target_include_directories(calibration_session_test PRIVATE "${REPO_ROOT}")
  add_test(NAME calibration_session COMMAND calibration_session_test)
endif()
