#pragma once
#include <cmath>
#include "Simd.h"

namespace vamp
{
// Newton step of the DK port equations  p = p0 + K i(p)  for devices in pairs of ports, as the tubes here: port 2t
// carries a current depending on ports 2t and 2t+1 (plate: Vpk, Vgk), port 2t+1 one depending on itself only (grid).
// With D = dI/dp, block-diagonal 2 x 2 per device ([[d[2t][0], d[2t][1]], [0, d[2t+1][1]]]), the step solves
//        (I - K D) dp = r.
// Written in the current changes q = D dp instead:  (I - D K) q = D r,  dp = r + K q.  A port whose row of D is all
// zero (a grid that does not conduct, a plate below 0 V) has q = 0, so only the ports that conduct are unknowns:
// 6 instead of 12 for six triodes whose grids draw no current, the usual case. The same solution as eliminating
// the full system (identical up to rounding); Gaussian elimination with partial pivoting.
// K: M x M row-major. False if the reduced matrix is singular.
// The active count is dispatched to a fixed-size instance (the same operations in the same order: only unrolled).
template <int M, int N>
inline bool solvePortStepN (const double* K, const double (&d)[M][2], const double (&r)[M], const int* act, double (&dp)[M]);

template <int M>
inline bool solvePortStep (const double* K, const double (&d)[M][2], const double (&r)[M], double (&dp)[M])
{
    int act[M];
    int n = 0;
    for (int k = 0; k < M; k += 2)
    {
        if (d[k][0] != 0.0 || d[k][1] != 0.0) act[n++] = k;
        if (d[k + 1][1] != 0.0) act[n++] = k + 1;
    }
    switch (n)
    {
        case 0:  for (int k = 0; k < M; ++k) dp[k] = r[k]; return true; // nothing conducts: dp = r
        case 1:  return solvePortStepN<M, 1> (K, d, r, act, dp);
        case 2:  return solvePortStepN<M, (M < 2 ? M : 2)> (K, d, r, act, dp);
        case 3:  return solvePortStepN<M, (M < 3 ? M : 3)> (K, d, r, act, dp);
        case 4:  return solvePortStepN<M, (M < 4 ? M : 4)> (K, d, r, act, dp);
        case 5:  return solvePortStepN<M, (M < 5 ? M : 5)> (K, d, r, act, dp);
        case 6:  return solvePortStepN<M, (M < 6 ? M : 6)> (K, d, r, act, dp);
        case 7:  return solvePortStepN<M, (M < 7 ? M : 7)> (K, d, r, act, dp);
        case 8:  return solvePortStepN<M, (M < 8 ? M : 8)> (K, d, r, act, dp);
        case 9:  return solvePortStepN<M, (M < 9 ? M : 9)> (K, d, r, act, dp);
        case 10: return solvePortStepN<M, (M < 10 ? M : 10)> (K, d, r, act, dp);
        case 11: return solvePortStepN<M, (M < 11 ? M : 11)> (K, d, r, act, dp);
        default: return solvePortStepN<M, M> (K, d, r, act, dp);
    }
}

// The entries of K the step reads for one set of conducting ports, gathered once: the set rarely changes from one
// sample to the next (grids start or stop conducting) and K only when the circuit is rebuilt (a pot moves). The
// owner keeps one per network; solvePortStep (..., cache, kVersion) refills it when the set or K changed.
template <int M>
struct PortStepCache
{
    static constexpr int MP = (M + 3) & ~3; // M rounded up to a multiple of 4: the AVX path reads whole vectors
    unsigned version = ~0u;
    unsigned mask = ~0u; // conducting ports, bit k = port k
    int n = 0;
    int act[M] {};
    double rowK[M][MP] {};  // [x][y] = K[act x][act y] (0 past n with VAMP_PORTSTEP_AVX)
    double rowKg[M][MP] {}; // [x][y] = K[act x + 1][act y] (plate rows: the grid's row)
    double colK[M][M] {};   // [k][y] = K[k][act y] (the scalar path's dp = r + K q)
    double colKT[M][MP] {}; // [y][k] = K[k][act y], k contiguous (the AVX path's; filled only with VAMP_PORTSTEP_AVX)
};

template <int M, int n>
inline bool solvePortStepCachedN (const PortStepCache<M>& c, const double (&d)[M][2], const double (&r)[M], double (&dp)[M]);

// The AVX version of the step (solvePortStepCachedAvxN, below) is not used unless VAMP_PORTSTEP_AVX is defined as 1:
// bit-identical (dsp_test portsolve) but no gain at these sizes (M 10-12, 5-7 conducting ports). Measured 2026-10-29
// on a Core Ultra 9 275HX P-core, 8-sample blocks (dsp_test block8), scalar -> AVX: power amp 20.8 -> 22.4 us,
// preamp RED 27.6 -> 27.8 us, the whole chain 96.5 -> 98.1 us; solve_profile's preamp step 462 -> 420 cycles,
// power amp step 236 -> 245. Four-lane vectors over 5-7 unknowns lose to the fully unrolled scalar code once the
// broadcasts, the padding stores and the vzeroupper between the vector and scalar phases are paid.
#ifndef VAMP_PORTSTEP_AVX
 #define VAMP_PORTSTEP_AVX 0
#endif
#if VAMP_SIMD_SSE2
template <int M, int n>
VAMP_AVX_FN inline bool solvePortStepCachedAvxN (const PortStepCache<M>& c, const double (&d)[M][2], const double (&r)[M], double (&dp)[M]);
#endif

template <int M>
inline bool solvePortStep (const double* K, const double (&d)[M][2], const double (&r)[M], double (&dp)[M],
                           PortStepCache<M>& c, unsigned kVersion)
{
    unsigned mask = 0;
    for (int k = 0; k < M; k += 2)
    {
        if (d[k][0] != 0.0 || d[k][1] != 0.0) mask |= 1u << k;
        if (d[k + 1][1] != 0.0) mask |= 1u << (k + 1);
    }
    if (mask != c.mask || kVersion != c.version)
    {
        c.mask = mask;
        c.version = kVersion;
        c.n = 0;
        for (int k = 0; k < M; ++k) if (mask & (1u << k)) c.act[c.n++] = k;
        for (int x = 0; x < c.n; ++x)
        {
            const int k = c.act[x];
            for (int y = 0; y < c.n; ++y)
            {
                c.rowK[x][y] = K[k * M + c.act[y]];
                c.rowKg[x][y] = k % 2 == 0 ? K[(k + 1) * M + c.act[y]] : 0.0;
            }
#if VAMP_PORTSTEP_AVX
            for (int y = c.n; y < PortStepCache<M>::MP; ++y) c.rowK[x][y] = c.rowKg[x][y] = 0.0; // whole vectors read
#endif
        }
        for (int k = 0; k < M; ++k)
            for (int y = 0; y < c.n; ++y) c.colK[k][y] = K[k * M + c.act[y]];
#if VAMP_PORTSTEP_AVX
        for (int y = 0; y < c.n; ++y)
            for (int k = 0; k < M; ++k) c.colKT[y][k] = K[k * M + c.act[y]];
#endif
    }
#if VAMP_PORTSTEP_AVX && VAMP_SIMD_SSE2
    if (vampHasAvx())
        switch (c.n)
        {
            case 0:  for (int k = 0; k < M; ++k) dp[k] = r[k]; return true;
            case 1:  return solvePortStepCachedAvxN<M, 1> (c, d, r, dp);
            case 2:  return solvePortStepCachedAvxN<M, (M < 2 ? M : 2)> (c, d, r, dp);
            case 3:  return solvePortStepCachedAvxN<M, (M < 3 ? M : 3)> (c, d, r, dp);
            case 4:  return solvePortStepCachedAvxN<M, (M < 4 ? M : 4)> (c, d, r, dp);
            case 5:  return solvePortStepCachedAvxN<M, (M < 5 ? M : 5)> (c, d, r, dp);
            case 6:  return solvePortStepCachedAvxN<M, (M < 6 ? M : 6)> (c, d, r, dp);
            case 7:  return solvePortStepCachedAvxN<M, (M < 7 ? M : 7)> (c, d, r, dp);
            case 8:  return solvePortStepCachedAvxN<M, (M < 8 ? M : 8)> (c, d, r, dp);
            case 9:  return solvePortStepCachedAvxN<M, (M < 9 ? M : 9)> (c, d, r, dp);
            case 10: return solvePortStepCachedAvxN<M, (M < 10 ? M : 10)> (c, d, r, dp);
            case 11: return solvePortStepCachedAvxN<M, (M < 11 ? M : 11)> (c, d, r, dp);
            default: return solvePortStepCachedAvxN<M, M> (c, d, r, dp);
        }
#endif
    switch (c.n)
    {
        case 0:  for (int k = 0; k < M; ++k) dp[k] = r[k]; return true;
        case 1:  return solvePortStepCachedN<M, 1> (c, d, r, dp);
        case 2:  return solvePortStepCachedN<M, (M < 2 ? M : 2)> (c, d, r, dp);
        case 3:  return solvePortStepCachedN<M, (M < 3 ? M : 3)> (c, d, r, dp);
        case 4:  return solvePortStepCachedN<M, (M < 4 ? M : 4)> (c, d, r, dp);
        case 5:  return solvePortStepCachedN<M, (M < 5 ? M : 5)> (c, d, r, dp);
        case 6:  return solvePortStepCachedN<M, (M < 6 ? M : 6)> (c, d, r, dp);
        case 7:  return solvePortStepCachedN<M, (M < 7 ? M : 7)> (c, d, r, dp);
        case 8:  return solvePortStepCachedN<M, (M < 8 ? M : 8)> (c, d, r, dp);
        case 9:  return solvePortStepCachedN<M, (M < 9 ? M : 9)> (c, d, r, dp);
        case 10: return solvePortStepCachedN<M, (M < 10 ? M : 10)> (c, d, r, dp);
        case 11: return solvePortStepCachedN<M, (M < 11 ? M : 11)> (c, d, r, dp);
        default: return solvePortStepCachedN<M, M> (c, d, r, dp);
    }
}

// solvePortStepN with K's entries from the cache: the same operations in the same order (bit-identical)
template <int M, int n>
inline bool solvePortStepCachedN (const PortStepCache<M>& c, const double (&d)[M][2], const double (&r)[M], double (&dp)[M])
{
    double a[n][n + 1];
    for (int x = 0; x < n; ++x)
    {
        const int k = c.act[x];
        const double* Kk = c.rowK[x];
        if (k % 2 == 0)
        {
            const double d0 = d[k][0], d1 = d[k][1];
            const double* Kg = c.rowKg[x];
            for (int y = 0; y < n; ++y) a[x][y] = -(d0 * Kk[y] + d1 * Kg[y]);
            a[x][n] = d0 * r[k] + d1 * r[k + 1];
        }
        else
        {
            const double d1 = d[k][1];
            for (int y = 0; y < n; ++y) a[x][y] = -d1 * Kk[y];
            a[x][n] = d1 * r[k];
        }
        a[x][x] += 1.0;
    }

    double q[n];
    double invPivot[n];
    for (int col = 0; col < n; ++col)
    {
        int piv = col;
        for (int x = col + 1; x < n; ++x)
            if (std::abs (a[x][col]) > std::abs (a[piv][col])) piv = x;
        if (std::abs (a[piv][col]) < 1e-300) return false;
        if (piv != col)
            for (int j = col; j <= n; ++j) { const double t = a[col][j]; a[col][j] = a[piv][j]; a[piv][j] = t; }
        invPivot[col] = 1.0 / a[col][col];
        for (int x = col + 1; x < n; ++x)
        {
            const double f = a[x][col] * invPivot[col];
            if (f == 0.0) continue;
            for (int j = col + 1; j <= n; ++j) a[x][j] -= f * a[col][j];
        }
    }
    for (int x = n - 1; x >= 0; --x)
    {
        double s = a[x][n];
        for (int j = x + 1; j < n; ++j) s -= a[x][j] * q[j];
        q[x] = s * invPivot[x];
    }

    for (int k = 0; k < M; ++k)
    {
        const double* Kk = c.colK[k];
        double s = r[k];
        for (int y = 0; y < n; ++y) s += Kk[y] * q[y];
        dp[k] = s;
    }
    return true;
}

#if VAMP_SIMD_SSE2
// solvePortStepCachedN four ports at a time (AVX; chosen at run time): the rows of (I - D K), the elimination's row
// updates and dp = r + K q as vectors over y / j / k. Separate multiplies and adds in the scalar path's order per
// element, so every result is bit-identical to it. The rows of a carry zeros past the last column (RL), so the
// vectors can run over the end; the last ports of dp (M not a multiple of 4) go as the scalar path.
// Vector and scalar work are kept in separate phases with _mm256_zeroupper() between them: this code is compiled
// without /arch:AVX, so its scalar parts are legacy SSE instructions, which pay a merge penalty on every one while
// the upper halves of the ymm registers are dirty (measured: a slower step than the scalar path when interleaved).
template <int M, int n>
VAMP_AVX_FN inline bool solvePortStepCachedAvxN (const PortStepCache<M>& c, const double (&d)[M][2], const double (&r)[M], double (&dp)[M])
{
    constexpr int RL = ((n + 1 + 3) & ~3) + 4; // n + 1 columns rounded up to 4, plus a vector of slack for the updates
    double a[n][RL];

    // 1. the rows of -D K (vectors over y; rowK / rowKg are MP wide with zeros past n)
    {
        const __m256d signMask = _mm256_set1_pd (-0.0);
        for (int x = 0; x < n; ++x)
        {
            const int k = c.act[x];
            const double* Kk = c.rowK[x];
            if (k % 2 == 0)
            {
                const __m256d d0 = _mm256_broadcast_sd (&d[k][0]), d1 = _mm256_broadcast_sd (&d[k][1]);
                const double* Kg = c.rowKg[x];
                for (int y = 0; y < n; y += 4)
                {
                    const __m256d t = _mm256_add_pd (_mm256_mul_pd (d0, _mm256_loadu_pd (Kk + y)), _mm256_mul_pd (d1, _mm256_loadu_pd (Kg + y)));
                    _mm256_storeu_pd (&a[x][y], _mm256_xor_pd (t, signMask)); // -(d0 Kk + d1 Kg)
                }
            }
            else
            {
                const __m256d d1 = _mm256_broadcast_sd (&d[k][1]);
                for (int y = 0; y < n; y += 4)
                    _mm256_storeu_pd (&a[x][y], _mm256_xor_pd (_mm256_mul_pd (d1, _mm256_loadu_pd (Kk + y)), signMask)); // -d1 Kk
            }
        }
        _mm256_zeroupper();
    }
    // 2. D r, the unit diagonal, the zero padding (scalar)
    for (int x = 0; x < n; ++x)
    {
        const int k = c.act[x];
        a[x][n] = k % 2 == 0 ? d[k][0] * r[k] + d[k][1] * r[k + 1] : d[k][1] * r[k];
        a[x][x] += 1.0;
        for (int j = n + 1; j < RL; ++j) a[x][j] = 0.0;
    }

    // 3. elimination: the pivot and the multipliers of a column scalar, then its row updates as vectors
    double q[n];
    double invPivot[n];
    for (int col = 0; col < n; ++col)
    {
        int piv = col;
        for (int x = col + 1; x < n; ++x)
            if (std::abs (a[x][col]) > std::abs (a[piv][col])) piv = x;
        if (std::abs (a[piv][col]) < 1e-300) return false;
        if (piv != col)
            for (int j = col; j <= n; ++j) { const double t = a[col][j]; a[col][j] = a[piv][j]; a[piv][j] = t; }
        invPivot[col] = 1.0 / a[col][col];
        double f[n];
        int rows[n], numRows = 0;
        for (int x = col + 1; x < n; ++x)
        {
            const double fx = a[x][col] * invPivot[col];
            if (fx == 0.0) continue;
            f[numRows] = fx;
            rows[numRows++] = x;
        }
        if (numRows == 0) continue;
        for (int i = 0; i < numRows; ++i)
        {
            const __m256d f4 = _mm256_broadcast_sd (&f[i]);
            double* ax = a[rows[i]];
            for (int j = col + 1; j <= n; j += 4) // the last vector runs into the zero padding
                _mm256_storeu_pd (ax + j, _mm256_sub_pd (_mm256_loadu_pd (ax + j), _mm256_mul_pd (f4, _mm256_loadu_pd (&a[col][j]))));
        }
        _mm256_zeroupper();
    }
    // 4. back substitution (scalar)
    for (int x = n - 1; x >= 0; --x)
    {
        double s = a[x][n];
        for (int j = x + 1; j < n; ++j) s -= a[x][j] * q[j];
        q[x] = s * invPivot[x];
    }

    // 5. dp = r + K q, four ports at a time, the last ports (M not a multiple of 4) scalar
    constexpr int M4 = M & ~3;
    for (int k = 0; k < M4; k += 4)
    {
        __m256d s = _mm256_loadu_pd (r + k);
        for (int y = 0; y < n; ++y) s = _mm256_add_pd (s, _mm256_mul_pd (_mm256_broadcast_sd (&q[y]), _mm256_loadu_pd (c.colKT[y] + k)));
        _mm256_storeu_pd (dp + k, s);
    }
    _mm256_zeroupper(); // back to SSE code without the transition penalty
    for (int k = M4; k < M; ++k)
    {
        const double* Kk = c.colK[k];
        double s = r[k];
        for (int y = 0; y < n; ++y) s += Kk[y] * q[y];
        dp[k] = s;
    }
    return true;
}
#endif

template <int M, int n>
inline bool solvePortStepN (const double* K, const double (&d)[M][2], const double (&r)[M], const int* act, double (&dp)[M])
{
    // rows of (I - D K) and D r over the active ports
    double a[n][n + 1];
    for (int x = 0; x < n; ++x)
    {
        const int k = act[x];
        const double* Kk = K + k * M;
        if (k % 2 == 0)
        {
            const double d0 = d[k][0], d1 = d[k][1];
            const double* Kg = Kk + M;
            for (int y = 0; y < n; ++y) a[x][y] = -(d0 * Kk[act[y]] + d1 * Kg[act[y]]);
            a[x][n] = d0 * r[k] + d1 * r[k + 1];
        }
        else
        {
            const double d1 = d[k][1];
            for (int y = 0; y < n; ++y) a[x][y] = -d1 * Kk[act[y]];
            a[x][n] = d1 * r[k];
        }
        a[x][x] += 1.0;
    }

    double q[n];
    double invPivot[n];
    for (int col = 0; col < n; ++col)
    {
        int piv = col;
        for (int x = col + 1; x < n; ++x)
            if (std::abs (a[x][col]) > std::abs (a[piv][col])) piv = x;
        if (std::abs (a[piv][col]) < 1e-300) return false;
        if (piv != col)
            for (int j = col; j <= n; ++j) { const double t = a[col][j]; a[col][j] = a[piv][j]; a[piv][j] = t; }
        invPivot[col] = 1.0 / a[col][col];
        for (int x = col + 1; x < n; ++x)
        {
            const double f = a[x][col] * invPivot[col];
            if (f == 0.0) continue;
            for (int j = col + 1; j <= n; ++j) a[x][j] -= f * a[col][j];
        }
    }
    for (int x = n - 1; x >= 0; --x)
    {
        double s = a[x][n];
        for (int j = x + 1; j < n; ++j) s -= a[x][j] * q[j];
        q[x] = s * invPivot[x];
    }

    for (int k = 0; k < M; ++k)
    {
        const double* Kk = K + k * M;
        double s = r[k];
        for (int y = 0; y < n; ++y) s += Kk[act[y]] * q[y];
        dp[k] = s;
    }
    return true;
}
} // namespace vamp
