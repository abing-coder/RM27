#ifndef IO__CAN_BACKEND_HPP
#define IO__CAN_BACKEND_HPP

#include <linux/can.h>

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
