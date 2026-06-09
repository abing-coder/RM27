// 极简 OpenMP 兼容桩（仅用于非 Linux 平台，如 macOS Apple clang 默认无 omp.h）。
//
// 本仓库实际并未使用任何 OpenMP 并行（无 #pragma omp / omp_* 调用），yolov8.cpp 仅 #include <omp.h>。
// 该桩头提供最常用的几个函数桩，保证编译/链接通过；如需真正的 OpenMP，请改装 libomp 并移除该兼容目录。
#ifndef MACOS_COMPAT_OMP_H
#define MACOS_COMPAT_OMP_H

#ifdef __cplusplus
extern "C" {
#endif

static inline int omp_get_thread_num(void) { return 0; }
static inline int omp_get_num_threads(void) { return 1; }
static inline int omp_get_max_threads(void) { return 1; }
static inline void omp_set_num_threads(int n) { (void)n; }

#ifdef __cplusplus
}
#endif

#endif  // MACOS_COMPAT_OMP_H
