#pragma once
#include "PowerAmp5150.h"
#include "Preamp5150.h"
#include "ModEq.h"

namespace vamp
{
// One sample of preamp -> post-gain (Master) -> power amp, with the preamp running on the power amp's supply
// rails: it gets V1+ / V2+ as they are now and reports what it draws, which sags them for the next sample.
// post: channel post-gain trim (AmpSim kChannelPost). eq: the EQ MOD between tone stack and power amp (nullptr:
// stock; the stack and the Master pot are linear, so after them is the same as between them). Returns volts across
// the speaker.
inline double processAmp5150 (Preamp5150& pre, PowerAmp5150& pa, double vin, double post, ModEq* eq = nullptr)
{
    pre.setRails (pa.getRailV1(), pa.getRailV2());
    double v = pre.process (vin) * post;
    if (eq) v = eq->process (v);
    double i1, i2;
    pre.railCurrents (i1, i2);
    pa.setPreampLoad (i1, i2);
    return pa.process (v);
}
} // namespace vamp
