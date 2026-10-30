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
inline unsigned long long cycles[64] {};
inline long long calls[64] {};

struct Scope
{
    int slot;
    unsigned long long t0;
    explicit Scope (int s) : slot (s), t0 (__rdtsc()) {}
    ~Scope() { cycles[slot] += __rdtsc() - t0; ++calls[slot]; }
};
// consecutive phases: each lap(slot) charges the time since the previous lap (or the start) to slot
struct Lap
{
    unsigned long long t = __rdtsc();
    void operator() (int slot) { const auto n = __rdtsc(); cycles[slot] += n - t; ++calls[slot]; t = n; }
};
} // namespace vamp::prof

#define VAMP_PROF_CAT2(a, b) a##b
#define VAMP_PROF_CAT(a, b) VAMP_PROF_CAT2 (a, b)
#define VAMP_PROF(slot) const vamp::prof::Scope VAMP_PROF_CAT (vampProf, __LINE__) (slot)
#define VAMP_PROF_LAPS vamp::prof::Lap vampLap
#define VAMP_PROF_LAP(slot) vampLap (slot)
#else
#define VAMP_PROF(slot)
#define VAMP_PROF_LAPS
#define VAMP_PROF_LAP(slot)
#endif

namespace vamp::prof
{
// slots: the 12AX7 network (TriodeNetwork::solve) and the power amp (PowerAmp5150::process)
enum Slot
{
    kTriSolve = 0, kTriP0, kTriEval, kTriResidual, kTriStep, kTriFinish, kTriCheck,
    kPowProcess = 10, kPowP0, kPowEval, kPowResidual, kPowStep, kPowFinish, kPowSupply, kPowCheck,
    // DKCircuit::rebuild (a pot moved)
    kDkRebuild = 20, kDkStamp, kDkInvert, kDkAinvNi, kDkV, kDkPMO, kDkTranspose, kDkUpdate,
    // SemiNetwork (the 1176's circuits: phases of one Newton evaluation and of a sample) and Urei1176::processSample
    kSemiP0 = 30, kSemiEval, kSemiResidual, kSemiJacobian, kSemiSolve, kSemiApply, kSemiFinish, kSemiOutputs, kSemiTrial,
    kCompSample = 40, kCompA, kCompC, kCompPasses, kCompB
};
}
