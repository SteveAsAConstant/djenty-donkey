#pragma once
// 2-wide double SIMD where the platform always has it: SSE2 on x86-64, NEON on arm64 (scalar code elsewhere), and
// on x86-64 4-wide AVX where the CPU has it (checked at run time: vampHasAvx()). Used only as separate multiplies
// and adds, so each lane computes exactly what the scalar code would, whichever path runs.
#if defined(__SSE2__) || defined(_M_X64) || defined(_M_AMD64) || defined(__x86_64__)
 #include <emmintrin.h>
 #include <immintrin.h>
 #define VAMP_SIMD_SSE2 1
 #if defined(_MSC_VER) && ! defined(__clang__)
  #include <intrin.h>
  #define VAMP_AVX_FN // MSVC compiles AVX intrinsics in any function
 #else
  #define VAMP_AVX_FN __attribute__ ((target ("avx")))
 #endif
namespace vamp
{
// AVX available (CPU and operating system)
inline bool vampHasAvx()
{
    static const bool has = []
    {
  #if defined(_MSC_VER) && ! defined(__clang__)
        int r[4];
        __cpuid (r, 1);
        const bool cpu = (r[2] & (1 << 28)) != 0 && (r[2] & (1 << 27)) != 0; // AVX, OSXSAVE
        return cpu && (_xgetbv (0) & 6) == 6;                                // XMM and YMM state enabled by the OS
  #else
        return __builtin_cpu_supports ("avx");
  #endif
    }();
    return has;
}
} // namespace vamp
#elif defined(__aarch64__) || defined(_M_ARM64)
 #include <arm_neon.h>
 #define VAMP_SIMD_NEON 1
#endif
