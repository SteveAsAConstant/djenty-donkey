#pragma once
#include "Filters.h"

namespace vamp
{
// EQ MOD (our addition, not on the amp). The 5150's passive tone stack moves little above noon: Middle 5 -> 10 adds
// 1.4 dB at 500 Hz, Bass / Treble 5 -> 10 +3..4 dB, while Middle 0 takes 6.7 dB out (factory network, pots as marked:
// HIGH 250K B, LOW 1M A, MID 50K B; a larger Mid pot gives +4.5 dB at most, docs/5150_schematic_notes.md). With the
// switch on, the Bass / Middle / Treble knobs also drive this active three-band EQ between the tone stack and the
// power amp: +-kRangeDb at 0 / 10, 0 dB at 5, linear in dB over the knob, on top of the stack's own change. At
// 5 / 5 / 5 every band is an exact identity (b = a), so the amp is the stock one. Ahead of the power amp, Presence,
// Resonance and the power amp's saturation respond to it as they do to the stack.
class ModEq
{
public:
    static constexpr double kRangeDb = 10.0;
    static constexpr double kBassHz = 100.0, kMidHz = 700.0, kMidQ = 0.8, kTrebleHz = 3000.0, kShelfQ = 0.707;

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        set (0.5, 0.5, 0.5, 0.0);
        reset();
    }
    void reset() { for (auto& b : band) b.reset(); }

    // the amp's knobs 0..1; depth 0..1: the switch, ramped by AmpSim's knob smoothing
    void set (double bass, double mid, double treble, double depth)
    {
        auto db = [depth] (double k) { return depth * kRangeDb * (2.0 * k - 1.0); };
        band[0].set (BiquadType::lowShelf, fs, kBassHz, kShelfQ, db (bass));
        band[1].set (BiquadType::peak, fs, kMidHz, kMidQ, db (mid));
        band[2].set (BiquadType::highShelf, fs, kTrebleHz, kShelfQ, db (treble));
    }

    double process (double x) { return band[2].process (band[1].process (band[0].process (x))); }

private:
    double fs = 48000.0;
    BiquadD band[3]; // double: the 100 Hz shelf at the oversampled rate (~350 kHz)
};
} // namespace vamp
