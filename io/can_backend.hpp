#ifndef IO__CAN_BACKEND_HPP
#define IO__CAN_BACKEND_HPP

#if defined(__linux__)
#include <linux/can.h>
#else
// 非 Linux 平台（如 macOS）没有 SocketCAN。这里提供与 <linux/can.h> 兼容的 can_frame
// 定义，仅用于让 CBoard 等代码能在非 Linux 上编译通过（视频调试场景），并不提供真实 CAN 功能。
#include <cstdint>
struct can_frame
{
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t __pad;
  uint8_t __res0;
  uint8_t __res1;
  uint8_t data[8];
};
#endif

namespace io
{
// SocketCAN 与 DamiaoCAN 的统一接口，供 CBoard 使用
struct ICan
{
  virtual void write(can_frame * frame) const = 0;
  virtual ~ICan() = default;
};
}  // namespace io

#endif
