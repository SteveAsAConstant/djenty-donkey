#pragma once
#include "PowerAmp5150.h"
#include "Preamp5150.h"

namespace vamp_baseline
{
// One sample of preamp -> post-gain (Master) -> power amp, with the preamp running on the power amp's supply
// rails: it gets V1+ / V2+ as they are now and reports what it draws, which sags them for the next sample.
// post: channel post-gain trim (AmpSim kChannelPost). Returns volts across the speaker.
inline double processAmp5150 (Preamp5150& pre, PowerAmp5150& pa, double vin, double post)
{
    pre.setRails (pa.getRailV1(), pa.getRailV2());
    const double v = pre.process (vin) * post;
    double i1, i2;
    pre.railCurrents (i1, i2);
    pa.setPreampLoad (i1, i2);
    return pa.process (v);
}
} // namespace vamp_baseline
