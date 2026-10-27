// fmt 枚举兼容 shim（无侵入）
//
// fmt 10 起移除了对枚举类型的隐式格式化（旧代码里大量 fmt::format("{}", some_enum) 会编译失败）。
// 本仓库在新版 fmt（如 macOS Homebrew 的 fmt 12）下编译时，通过该头为所有枚举恢复
// “按底层整数格式化”的旧行为，从而无需改动任何业务源码。
//
// 该头通过编译选项 -include 被自动注入到每个 C++ 翻译单元（见根 CMakeLists.txt），
// 用 __has_include 保护，对不依赖 fmt 的目标无副作用。
#pragma once

#if __has_include(<fmt/format.h>)
#include <fmt/format.h>

#include <type_traits>

template <typename E, typename Char>
struct fmt::formatter<E, Char, std::enable_if_t<std::is_enum_v<E>>>
: fmt::formatter<std::underlying_type_t<E>, Char>
{
  template <typename FormatContext>
  auto format(E value, FormatContext & ctx) const
  {
    return fmt::formatter<std::underlying_type_t<E>, Char>::format(
      static_cast<std::underlying_type_t<E>>(value), ctx);
  }
};
#endif
