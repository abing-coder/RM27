#include "io/damiao_can.hpp"

#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include <algorithm>
#include <cctype>

#include "tools/logger.hpp"

namespace io
{
// SDK 回调无 user_data，单例转发（CBoard 仅一个 CAN 实例）
static DamiaoCAN * s_rec_instance = nullptr;

namespace
{
std::string trim_copy(const std::string & s)
{
  auto begin = std::find_if_not(s.begin(), s.end(), [](unsigned char ch) {return std::isspace(ch);});
  auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char ch) {return std::isspace(ch);}).base();
  if (begin >= end) return std::string();
  return std::string(begin, end);
}

int parse_human_device_ordinal(const std::string & raw, const char * source)
{
  const auto text = trim_copy(raw);
  if (text.empty()) {
    throw std::runtime_error(std::string("DamiaoCAN: empty ") + source);
  }

  int ordinal = 0;
  try {
    ordinal = std::stoi(text);
  } catch (...) {
    throw std::runtime_error(std::string("DamiaoCAN: invalid ") + source + ", expected positive integer");
  }

  if (ordinal <= 0) {
    throw std::runtime_error(std::string("DamiaoCAN: invalid ") + source + ", expected >= 1");
  }
  return ordinal;
}
}  // namespace

DamiaoCAN::DamiaoCAN(const std::string & selector, std::function<void(const can_frame &)> rx_handler)
: rx_handler_(std::move(rx_handler))
{
  handle_ = damiao_handle_create(DEV_USB2CANFD);
  if (!handle_) {
    throw std::runtime_error("DamiaoCAN: damiao_handle_create failed");
  }

  damiao_print_version(static_cast<damiao_handle *>(handle_));

  int device_cnt = damiao_handle_find_devices(static_cast<damiao_handle *>(handle_));
  if (device_cnt == 0) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    throw std::runtime_error("DamiaoCAN: no CAN device found");
  }

  device_handle * dev_list[16];
  int handle_cnt = 0;
  damiao_handle_get_devices(
    static_cast<damiao_handle *>(handle_), dev_list, &handle_cnt);
  if (handle_cnt == 0) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    throw std::runtime_error("DamiaoCAN: damiao_handle_get_devices failed");
  }

  int device_ordinal = 1;  // 1-based: 第1个设备
  if (!selector.empty()) {
    const std::string prefix = "damiao:";
    if (selector.rfind(prefix, 0) == 0) {
      device_ordinal = parse_human_device_ordinal(selector.substr(prefix.size()), "selector") ;
    }
  }

  if (const char * env = std::getenv("DM_CAN_DEVICE_INDEX")) {
    device_ordinal = parse_human_device_ordinal(env, "DM_CAN_DEVICE_INDEX");
  }

  const int device_index = device_ordinal - 1;
  if (device_index < 0 || device_index >= handle_cnt || !dev_list[device_index]) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    throw std::runtime_error(
            "DamiaoCAN: device index out of range (requested ordinal=" +
            std::to_string(device_ordinal) + ", detected=" + std::to_string(handle_cnt) + ")");
  }

  dev_ = dev_list[device_index];

  if (!device_open(static_cast<device_handle *>(dev_))) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    dev_ = nullptr;
    throw std::runtime_error("DamiaoCAN: device_open failed");
  }

  if (!device_open_channel(static_cast<device_handle *>(dev_), 0)) {
    device_close(static_cast<device_handle *>(dev_));
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
    handle_ = nullptr;
    dev_ = nullptr;
    throw std::runtime_error("DamiaoCAN: device_open_channel(0) failed");
  }

  s_rec_instance = this;
  device_hook_to_rec(static_cast<device_handle *>(dev_), rec_callback_static);

  opened_ = true;
  tools::logger()->info(
    "DamiaoCAN opened (DM-USB2FDCAN SDK), device_ordinal={}, device_index_zero_based={}, total_devices={}",
    device_ordinal,
    device_index,
    handle_cnt);
}

DamiaoCAN::~DamiaoCAN()
{
  s_rec_instance = nullptr;
  if (opened_ && dev_) {
    device_close_channel(static_cast<device_handle *>(dev_), 0);
    device_close(static_cast<device_handle *>(dev_));
  }
  if (handle_) {
    damiao_handle_destroy(static_cast<damiao_handle *>(handle_));
  }
  handle_ = nullptr;
  dev_ = nullptr;
  opened_ = false;
  tools::logger()->info("DamiaoCAN destructed.");
}

void DamiaoCAN::write(can_frame * frame) const
{
  if (!opened_ || !dev_) throw std::runtime_error("DamiaoCAN: device not open");

  uint8_t payload[8];
  std::memcpy(payload, frame->data, frame->can_dlc <= 8 ? frame->can_dlc : 8);

  device_channel_send_fast(
    static_cast<device_handle *>(const_cast<void *>(dev_)),
    0,
    frame->can_id,
    1,
    false,
    false,
    false,
    frame->can_dlc <= 8 ? frame->can_dlc : 8,
    payload);
}

void DamiaoCAN::rec_callback_static(usb_rx_frame_t * rec_frame)
{
  if (s_rec_instance && rec_frame) s_rec_instance->on_rec(rec_frame);
}

void DamiaoCAN::on_rec(usb_rx_frame_t * rec_frame)
{
  can_frame kf;
  std::memset(&kf, 0, sizeof(kf));
  kf.can_id = rec_frame->head.can_id;
  kf.can_dlc = rec_frame->head.dlc <= 8 ? rec_frame->head.dlc : 8;
  std::memcpy(kf.data, rec_frame->payload, kf.can_dlc);

  if (rx_handler_) rx_handler_(kf);
}

}  // namespace io
