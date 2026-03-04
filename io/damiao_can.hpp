#ifndef IO__DAMIAO_CAN_HPP
#define IO__DAMIAO_CAN_HPP

#include "io/can_backend.hpp"

#include <linux/can.h>

#include <cstddef>
#include "io/damiao/pub_user.h"

#include <cstring>
#include <functional>
#include <memory>
#include <string>

namespace io
{
// 使用达妙官方 SDK 直接访问 DM-USB2FDCAN，无需 slcand/slcan0
class DamiaoCAN : public ICan
{
public:
  DamiaoCAN(const std::string &, std::function<void(const can_frame &)> rx_handler);
  ~DamiaoCAN() override;

  void write(can_frame * frame) const override;

private:
  static void rec_callback_static(usb_rx_frame_t * rec_frame);
  void on_rec(usb_rx_frame_t * rec_frame);

  std::function<void(const can_frame &)> rx_handler_;
  void * handle_ = nullptr;   // damiao_handle*
  void * dev_ = nullptr;      // device_handle*
  bool opened_ = false;
};

}  // namespace io

#endif
