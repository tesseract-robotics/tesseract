#pragma once

// SIMD-accelerated primitives for hot loops.
//
// Compile-time gated on TINYVDB_SIMD (CMake option) and compiler-provided
// feature defines (__SSE2__, __AVX2__, __F16C__). When TINYVDB_SIMD is OFF
// or the relevant feature is unavailable, each function falls back to a
// scalar reference implementation. Scalar and SIMD paths produce identical
// results modulo IEEE-754 reduction-order differences (dot product).
//
// Public names:
//   tvdb_simd_dot_f32      - dot product of two fp32 buffers, returns double
//   tvdb_simd_axpy_f32     - y[i] += alpha * x[i]
//   tvdb_simd_f16_to_f32   - bulk half -> float conversion (F16C if available)
//   tvdb_simd_f32_to_f16   - bulk float -> half conversion (round-to-nearest)
//
// Behavior is independent of OpenMP (TINYVDB_OPENMP); the two are orthogonal.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(TINYVDB_SIMD) && (defined(__AVX2__) || defined(__SSE2__))
#include <immintrin.h>
#endif

// ----- fp32 dot product (returns double for stability) -----

static inline double tvdb_simd_dot_f32(const float* a, const float* b, size_t n)
{
#if defined(TINYVDB_SIMD) && defined(__AVX2__)
  // 8-wide AVX2 reduction, accumulating into a double scalar each chunk
  // for numerical stability comparable to the scalar path.
  __m256 acc = _mm256_setzero_ps();
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
  {
    __m256 va = _mm256_loadu_ps(a + i);
    __m256 vb = _mm256_loadu_ps(b + i);
    acc = _mm256_add_ps(acc, _mm256_mul_ps(va, vb));
  }
  // Horizontal sum.
  __m128 lo = _mm256_castps256_ps128(acc);
  __m128 hi = _mm256_extractf128_ps(acc, 1);
  __m128 s = _mm_add_ps(lo, hi);
  s = _mm_hadd_ps(s, s);
  s = _mm_hadd_ps(s, s);
  float partial;
  _mm_store_ss(&partial, s);
  double total = (double)partial;
  for (; i < n; ++i)
    total += (double)a[i] * (double)b[i];
  return total;
#else
  double total = 0.0;
  for (size_t i = 0; i < n; ++i)
    total += (double)a[i] * (double)b[i];
  return total;
#endif
}

// ----- y[i] += alpha * x[i] -----

static inline void tvdb_simd_axpy_f32(float alpha, const float* x, float* y, size_t n)
{
#if defined(TINYVDB_SIMD) && defined(__AVX2__)
  __m256 va = _mm256_set1_ps(alpha);
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
  {
    __m256 vx = _mm256_loadu_ps(x + i);
    __m256 vy = _mm256_loadu_ps(y + i);
// FMA preferred when AVX2 implies FMA (it usually does on Haswell+).
#if defined(__FMA__)
    vy = _mm256_fmadd_ps(vx, va, vy);
#else
    vy = _mm256_add_ps(vy, _mm256_mul_ps(vx, va));
#endif
    _mm256_storeu_ps(y + i, vy);
  }
  for (; i < n; ++i)
    y[i] += alpha * x[i];
#else
  for (size_t i = 0; i < n; ++i)
    y[i] += alpha * x[i];
#endif
}

// ----- bulk half -> float -----

#if defined(TINYVDB_SIMD) && defined(__F16C__) && defined(__AVX2__)
// Hardware path: VCVTPH2PS converts 8 halves to 8 floats per instruction.
#define TVDB_SIMD_HAS_F16C 1
#else
#define TVDB_SIMD_HAS_F16C 0
#endif

// Scalar half<->float reference (matches IEEE 754 binary16, no NaN/Inf
// special-casing — adequate for VDB grid values).
static inline float tvdb__half_to_float_scalar(uint16_t h)
{
  uint32_t s = (uint32_t)(h & 0x8000u) << 16;
  uint32_t e = (h >> 10) & 0x1Fu;
  uint32_t m = h & 0x3FFu;
  uint32_t f;
  if (e == 0)
  {
    if (m == 0)
    {
      f = s;
    }
    else
    {
      // Subnormal half: normalize.
      int shift = 0;
      while ((m & 0x400u) == 0)
      {
        m <<= 1;
        ++shift;
      }
      m &= 0x3FFu;
      f = s | ((127u - 15u - shift + 1u) << 23) | (m << 13);
    }
  }
  else if (e == 31)
  {
    f = s | 0x7F800000u | (m << 13);  // Inf or NaN
  }
  else
  {
    f = s | ((e + 127u - 15u) << 23) | (m << 13);
  }
  union
  {
    uint32_t u;
    float f;
  } cv;
  cv.u = f;
  return cv.f;
}

static inline uint16_t tvdb__float_to_half_scalar(float v)
{
  union
  {
    uint32_t u;
    float f;
  } cv;
  cv.f = v;
  uint32_t u = cv.u;
  uint16_t s = (uint16_t)((u >> 16) & 0x8000u);
  int32_t e = (int32_t)((u >> 23) & 0xFFu) - 127 + 15;
  uint32_t m = u & 0x7FFFFFu;
  if (e <= 0)
  {
    // Subnormal or zero.
    if (e < -10)
      return s;  // underflow to zero
    m |= 0x800000u;
    uint16_t hm = (uint16_t)(m >> (14 - e));
    // Round to nearest even.
    if ((m >> (13 - e)) & 1u)
      ++hm;
    return s | hm;
  }
  else if (e >= 31)
  {
    // Overflow / Inf / NaN.
    return s | 0x7C00u | (uint16_t)(m ? (m >> 13) : 0u);
  }
  else
  {
    uint16_t hm = (uint16_t)((e << 10) | (m >> 13));
    // Round to nearest even.
    if (m & 0x1000u)
    {
      ++hm;  // may bump into next exp; acceptable
    }
    return s | hm;
  }
}

static inline void tvdb_simd_f16_to_f32(const uint16_t* in, float* out, size_t n)
{
#if TVDB_SIMD_HAS_F16C
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
  {
    __m128i h = _mm_loadu_si128((const __m128i*)(in + i));
    __m256 f = _mm256_cvtph_ps(h);
    _mm256_storeu_ps(out + i, f);
  }
  for (; i < n; ++i)
    out[i] = tvdb__half_to_float_scalar(in[i]);
#else
  for (size_t i = 0; i < n; ++i)
    out[i] = tvdb__half_to_float_scalar(in[i]);
#endif
}

static inline void tvdb_simd_f32_to_f16(const float* in, uint16_t* out, size_t n)
{
#if TVDB_SIMD_HAS_F16C
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
  {
    __m256 f = _mm256_loadu_ps(in + i);
    __m128i h = _mm256_cvtps_ph(f, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
    _mm_storeu_si128((__m128i*)(out + i), h);
  }
  for (; i < n; ++i)
    out[i] = tvdb__float_to_half_scalar(in[i]);
#else
  for (size_t i = 0; i < n; ++i)
    out[i] = tvdb__float_to_half_scalar(in[i]);
#endif
}

#ifdef __cplusplus
}
#endif
