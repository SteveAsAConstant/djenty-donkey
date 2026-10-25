#pragma once
#include <cmath>

namespace vamp
{
// Audio-taper (log) pot: 50% rotation -> ~10% resistance
inline double taperA (double x) { return (std::pow (10.0, 2.0 * x) - 1.0) / 99.0; }

// Peavey POT0752 "1M 5% A" (VR1 / VR2 pre-gain, VR8 Resonance), also used for the post-gain (Master) pots:
// a steeper log taper than taperA, 50% rotation -> ~4.5%. Calibrated against NAM captures of a Peavey 6505 1992
// Original (lead channel, pre-gain 2..5, post-gain 3; tests/nam_compare.cpp --fit, docs/nam_calibration.md): with
// taperA the model was ~7 dB hot at every dial position. The post-gain: v0.4 scaled it by 0.229 (~-13 dB) because
// the tone stack was driven from a stiff source; the stack is now fed through R89 470k / R24 33k as on the
// schematic, and that ~-24 dB divider is the real attenuation (a drive-only fit to all 14 captures is consistent
// with no extra scale). The knobs read like the real amp's dials.
constexpr double kPotTaperK = 2.65;
inline double taperPot (double x) { return (std::pow (10.0, kPotTaperK * x) - 1.0) / (std::pow (10.0, kPotTaperK) - 1.0); }
inline double taperPotInverse (double f) { return std::log10 (1.0 + f * (std::pow (10.0, kPotTaperK) - 1.0)) / kPotTaperK; }
} // namespace vamp
