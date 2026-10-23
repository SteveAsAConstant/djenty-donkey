#pragma once
#include <algorithm>
#include <cmath>
#include "LinearNetwork.h"
#include "TriodeStage.h"

namespace vamp
{
// Audio-taper (log) pot: 50% rotation -> ~10% resistance
inline double taperA (double x) { return (std::pow (10.0, 2.0 * x) - 1.0) / 99.0; }

// Component-level model of the EVH 5150 / 6505 preamp + tone stack.
// Designators refer to docs/5150_schematic_notes.md.
//
//  RED   (5150 lead):     V1A -> lead pre-gain net -> V1B -> V2A -> V2B -> V5B -> V5A -> tone stack
//  BLUE  (6505 crunch):   V1A -> rhythm net (crunch) -> V1B (rhythm cathode) -> V2A (R20 shunt) -> ... -> tone stack
//  CLEAN (6505 rhythm, optimized): V1A -> rhythm net (clean shunt) -> V1B -> tone stack
//         "optimized" = V2A..V5A bypassed for headroom (not in the original amp)
class Preamp5150
{
public:
    enum Mode { Clean = 0, Blue = 1, Red = 2 };

    double vb1 = 320.0, vb2 = 320.0; // preamp B+ (not on the schematic; forum ballpark 300-350 V)

    TriodeStage v1a, v1b, v2a, v2b, v5b, v5a;

    Preamp5150()
    {
        // ---- lead pre-gain network: C6 -> R29||C17 -> R41 / VR2 (+ C19||R40) ----
        leadNet.addR (SRC, 3, kRoutV1A);   // V1A plate output impedance
        leadNet.addC (3, 0, 22e-9);        // C6
        leadNet.addR (0, 1, 470e3);        // R29
        leadNet.addC (0, 1, 2.2e-9);       // C17
        leadNet.addR (1, GND, 1e6);        // R41
        leadTop = leadNet.addR (1, 2, 5e5);// VR2 upper
        leadBot = leadNet.addR (2, GND, 5e5); // VR2 lower
        leadNet.addC (1, 2, 1e-9);         // C19
        leadNet.addR (1, 2, 1e6);          // R40

        // ---- rhythm pre-gain network: C6 -> R19||C51 -> [R23+C53 clean shunt] -> VR1 (+C27) ----
        rhythmNet.addR (SRC, 4, kRoutV1A); // V1A plate output impedance
        rhythmNet.addC (4, 0, 22e-9);      // C6
        rhythmNet.addR (0, 1, 470e3);      // R19
        rhythmNet.addC (0, 1, 270e-12);    // C51
        rhyShunt = rhythmNet.addR (1, 3, 39e3); // R23 (switched out for crunch)
        rhythmNet.addC (3, GND, 0.1e-6);   // C53
        rhyTop = rhythmNet.addR (1, 2, 5e5); // VR1 upper
        rhyBot = rhythmNet.addR (2, GND, 5e5); // VR1 lower
        rhythmNet.addC (1, 2, 180e-12);    // C27 bright

        // ---- tone stack (Marshall-style TMB): nodes 0 trebTop, 1 out, 2 trebBot, 3 x, 4 midTop ----
        toneStack.addC (SRC, 0, 470e-12);  // C14 treble cap
        tsTrebUp = toneStack.addR (0, 1, 125e3); // VR5 treble 250k upper
        tsTrebLo = toneStack.addR (1, 2, 125e3); // VR5 lower
        toneStack.addR (SRC, 3, 47e3);     // R28 slope
        toneStack.addC (3, 2, 22e-9);      // C18 bass cap
        toneStack.addC (3, 4, 22e-9);      // C21 mid cap
        tsBass = toneStack.addR (2, 4, 5e5);  // VR3 bass 1M (rheostat)
        tsMid = toneStack.addR (4, GND, 25e3); // VR4 mid 50k (rheostat)
        toneStack.addR (1, GND, 1e6);      // load: post-gain pot / R26
    }

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        leadNet.prepare (fs);
        rhythmNet.prepare (fs);
        toneStack.prepare (fs);
        configureStages();

        // let coupling / cathode caps settle
        for (int i = 0; i < (int) (fs * 0.3); ++i)
            process (0.0);
    }

    void setMode (int newMode)
    {
        if (newMode == mode) return;
        mode = newMode;
        configureStages();
    }

    // all 0..1 knob positions
    void setControls (double preGain, double bass, double mid, double treble, double post)
    {
        setPreGainPot (taperA (preGain));
        setToneStackRaw (std::clamp (treble, 0.0, 1.0), std::clamp (mid, 0.0, 1.0),
                         taperA (std::clamp (bass, 0.0, 1.0)));
        postGain = taperA (std::clamp (post, 0.0, 1.0));
    }

    // raw pot fractions (treble wiper from bottom, mid / bass rheostat fraction)
    void setToneStackRaw (double t, double m, double l)
    {
        toneStack.setR (tsTrebUp, (1.0 - t) * 250e3);
        toneStack.setR (tsTrebLo, t * 250e3);
        toneStack.setR (tsBass, l * 1e6);
        toneStack.setR (tsMid, m * 50e3);
    }

    double processToneStack (double x) { return toneStack.process (x, 1); }

    // raw pot fraction (wiper position from ground), used by the tests
    void setPreGainPot (double f)
    {
        f = std::clamp (f, 0.0, 1.0);
        leadNet.setR (leadTop, (1.0 - f) * 1e6);
        leadNet.setR (leadBot, f * 1e6);
        rhythmNet.setR (rhyTop, (1.0 - f) * 1e6);
        rhythmNet.setR (rhyBot, f * 1e6);
    }

    // input: guitar voltage. output: volts after tone stack and post-gain pot
    double process (double vin)
    {
        const double a = v1a.process (vin);
        double x;

        if (mode == Red)
        {
            x = v1b.process (leadNet.process (a, 2));
            x = v2a.process (x);
            x = v2b.process (x);
            x = v5b.process (x);
            x = v5a.process (x);
        }
        else
        {
            x = v1b.process (rhythmNet.process (a, 2));
            if (mode == Blue)
            {
                x = v2a.process (x);
                x = v2b.process (x);
                x = v5b.process (x);
                x = v5a.process (x);
            }
        }

        lastPreampOut = x;
        return toneStack.process (x, 1) * postGain;
    }

    double getLastPreampOut() const { return lastPreampOut; }
    int getMode() const { return mode; }

private:
    void configureStages()
    {
        const bool lead = mode == Red;

        rhythmNet.setR (rhyShunt, mode == Clean ? 39e3 : 1e12); // crunch removes the R23/C53 shunt

        TriodeConfig c;

        // V1A: input jack R27 22k -> C15 0.1u -> R25 1M -> R22 68k
        c = {}; c.vb = vb1; c.rsrc = 22e3; c.cc = 0.1e-6; c.ra = 1e6; c.rs = 68e3;
        c.rp = 220e3; c.rk = 1.82e3; c.ck = 1e-6;
        v1a.configure (c, fs);

        // V1B: R82 470k grid feed; C26 22u fully (lead) or via R18 47k (rhythm)
        c = {}; c.vb = vb1; c.rsrc = 470e3; c.rs = 1e3;
        c.rp = 100e3; c.rk = 1.82e3; c.ck = 1e-6; c.ck2 = 22e-6; c.rk2 = lead ? 1.0 : 47e3;
        v1b.configure (c, fs);

        // V2A: C2 22n -> R6 470k -> grid, R9 1M (|| R20 220k in rhythm); 39k cold cathode
        c = {}; c.vb = vb1; c.rsrc = kRoutV1B; c.cc = 22e-9; c.rs = 470e3; c.rb = lead ? 1e6 : (1e6 * 220e3) / (1e6 + 220e3);
        c.rp = 100e3; c.rk = 39e3;
        v2a.configure (c, fs);

        // V2B: C1 22n -> C48 1n || R11 330k -> R7 220k -> grid
        c = {}; c.vb = vb2; c.rsrc = kRoutV2A; c.cc = 22e-9; c.ra = 330e3; c.caShunt = 1e-9; c.rs = 220e3;
        c.rp = 220e3; c.rk = 1.82e3; c.ck = 1e-6;
        v2b.configure (c, fs);

        // V5B: R32 1M -> C57 22n -> R101 100k; 2.2k unbypassed
        c = {}; c.vb = vb2; c.rsrc = 1e6 + kRoutV2B; c.cc = 22e-9; c.ra = 100e3; c.rs = 1e3;
        c.rp = 220e3; c.rk = 2.2e3;
        v5b.configure (c, fs);

        // V5A: R87 1M -> C56 22n -> R91 1M, R88 1M plate->grid feedback; 2.2k unbypassed
        c = {}; c.vb = vb2; c.rsrc = 1e6 + kRoutV5B; c.cc = 22e-9; c.ra = 1e6; c.rs = 1e3;
        c.rp = 100e3; c.rk = 2.2e3; c.rfb = 1e6;
        v5a.configure (c, fs);
    }

    // Thevenin output impedance of each driving plate: Rp || (rp + (mu+1) Rk_unbypassed), rp ~ 62k
    static constexpr double kRoutV1A = 220e3 * 62e3 / (220e3 + 62e3);
    static constexpr double kRoutV1B = 100e3 * 62e3 / (100e3 + 62e3);
    static constexpr double kRoutV2A = 100e3 * (62e3 + 101 * 39e3) / (100e3 + 62e3 + 101 * 39e3);
    static constexpr double kRoutV2B = 220e3 * 62e3 / (220e3 + 62e3);
    static constexpr double kRoutV5B = 220e3 * (62e3 + 101 * 2.2e3) / (220e3 + 62e3 + 101 * 2.2e3);

    double fs = 192000.0;
    int mode = Red;
    double postGain = 0.1;
    double lastPreampOut = 0.0;

    LinearNetwork<4> leadNet;
    LinearNetwork<5> rhythmNet;
    LinearNetwork<5> toneStack;
    int leadTop = 0, leadBot = 0, rhyTop = 0, rhyBot = 0, rhyShunt = 0;
    int tsTrebUp = 0, tsTrebLo = 0, tsBass = 0, tsMid = 0;
};

} // namespace vamp
