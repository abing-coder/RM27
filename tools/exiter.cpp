#include "exiter.hpp"

#include <csignal>
#include <cstdlib>
#include <stdexcept>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false;

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");
  // 第二次 Ctrl+C 强制退出：设备析构（如 CAN SDK 关闭）卡住时仍能结束进程
  std::signal(SIGINT, [](int) {
    if (exit_) std::_Exit(130);
    exit_ = true;
  });
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

}  // namespace tools