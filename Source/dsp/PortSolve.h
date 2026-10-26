#pragma once
#include <cmath>

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
