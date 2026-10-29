#pragma once
// Cycle counters for finding where a solve's time goes (tests/solve_profile.cpp, built with VAMP_PROFILE). Without
// VAMP_PROFILE the macros are empty: no effect on the plugin or the other tests.
#ifdef VAMP_PROFILE
#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <x86intrin.h>
#endif

namespace vamp::prof
{
inline unsigned long long cycles[32] {};
inline long long calls[32] {};

struct Scope
{
    int slot;
    unsigned long long t0;
    explicit Scope (int s) : slot (s), t0 (__rdtsc()) {}
    ~Scope() { cycles[slot] += __rdtsc() - t0; ++calls[slot]; }
};
} // namespace vamp::prof

#define VAMP_PROF_CAT2(a, b) a##b
#define VAMP_PROF_CAT(a, b) VAMP_PROF_CAT2 (a, b)
#define VAMP_PROF(slot) const vamp::prof::Scope VAMP_PROF_CAT (vampProf, __LINE__) (slot)
#else
#define VAMP_PROF(slot)
#endif

namespace vamp::prof
{
// slots: the 12AX7 network (TriodeNetwork::solve) and the power amp (PowerAmp5150::process)
enum Slot
{
    kTriSolve = 0, kTriP0, kTriEval, kTriResidual, kTriStep, kTriFinish, kTriCheck,
    kPowProcess = 10, kPowP0, kPowEval, kPowResidual, kPowStep, kPowFinish, kPowSupply, kPowCheck
};
}
