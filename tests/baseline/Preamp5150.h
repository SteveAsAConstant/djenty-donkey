#pragma once
#include <algorithm>
#include <cmath>
#include "LinearNetwork.h"
#include "LateStages5150.h"
#include "LeadPreamp5150.h"
#include "TriodeStage.h"

namespace vamp_baseline
{
// Audio-taper (log) pot: 50% rotation -> ~10% resistance
inline double taperA (double x) { return (std::pow (10.0, 2.0 * x) - 1.0) / 99.0; }

// Pre-gain and post-gain (Master) pots, calibrated against NAM captures of a Peavey 6505 1992
// Original (lead channel, pre-gain 2..5, post-gain 3; tests/nam_compare.cpp --fit, docs/nam_calibration.md):
// - pre-gain: a steeper log taper than taperA, 50% rotation -> ~4.5% (the fit put the model ~7 dB hot
//   at every dial position with taperA)
// - post-gain (Master): the same taper. v0.4 scaled it by 0.229 (~-13 dB) because the tone stack was driven from a
//   stiff source; the stack is now fed through R89 470k / R24 33k as on the schematic, and that ~-24 dB divider
//   is the real attenuation (a drive-only fit to all 14 captures is consistent with no extra scale).
// The knobs read like the real amp's dials.
constexpr double kPotTaperK = 2.65;
constexpr double kPostTrimV04 = 0.229; // v0.4's Master scale: keeps old presets on the same knob positions
inline double taperPot (double x) { return (std::pow (10.0, kPotTaperK * x) - 1.0) / (std::pow (10.0, kPotTaperK) - 1.0); }
inline double taperPotInverse (double f) { return std::log10 (1.0 + f * (std::pow (10.0, kPotTaperK) - 1.0)) / kPotTaperK; }
inline double masterTaper (double x) { return x <= 0.0 ? 0.0 : taperPot (x); }
inline double masterKnobForDrive (double drive) { return taperPotInverse (std::clamp (drive, 0.0, 1.0)); }

// Lead pre-gain: taperPot lifted below dial 5. Pre-gain fits (power-amp drive fixed at the Master calibration,
// nam_compare --fit-pre) to all 14 captures want +5.5 / +2.7 / +3.4 dB re taperPot at dials 2 / 3 / 4 and none at
// 5 (means of 4 / 5 / 3 / 2 captures, spread +-0.4..0.6 dB; +-1.2 at 5); interpolated linearly in dB between
// them, held below 2 (still monotonic: between 3 and 4 the lift grows 0.7 dB, taperPot 6 dB). The v0.4 curve was
// fitted to dials 3..5 with the stages solved one after another, which overstated V5B's swing. Nothing above 5
// was captured, so from dial 5 up the curve is unchanged. Lead only: the captures are lead.
inline double leadPreLiftDb (double x)
{
    constexpr double at[4] = { 5.47, 2.74, 3.43, 0.0 }; // dials 2, 3, 4, 5
    if (x >= 0.5) return 0.0;
    if (x <= 0.2) return at[0];
    const double f = (x - 0.2) * 10.0;
    const int i = std::min (2, (int) f);
    return at[i] + (f - i) * (at[i + 1] - at[i]);
}
inline double leadPreTaper (double x)
{
    return std::min (1.0, taperPot (x) * std::pow (10.0, leadPreLiftDb (x) / 20.0));
}
inline double leadPreTaperInverse (double f) // monotonic: bisection (not used on the audio thread)
{
    double lo = 0.0, hi = 1.0;
    for (int i = 0; i < 60; ++i) { const double mid = 0.5 * (lo + hi); (leadPreTaper (mid) < f ? lo : hi) = mid; }
    return 0.5 * (lo + hi);
}

// v0.3 knob curves (pre-gain taperA, Master 60 dB log), to convert old presets / sessions
inline double masterTaperV03 (double x) { return x <= 0.0 ? 0.0 : std::pow (10.0, 3.0 * (x - 1.0)) * std::min (1.0, x * 20.0); }
inline double gainKnobFromV03 (double x, bool lead) { return lead ? leadPreTaperInverse (taperA (x)) : taperPotInverse (taperA (x)); }
// (v0.3 -> v0.4 knob positions, which v0.5 keeps)
inline double masterKnobFromV03 (double x) { return taperPotInverse (std::clamp (masterTaperV03 (x) / kPostTrimV04, 0.0, 1.0)); }

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

    // idle preamp B+ (V1+, V2+): the DC solution of the factory supply chain (PowerAmp5150, schematic sheet 2) with
    // the model's tube currents. With the supply connected (setRails) they follow the sagging rails per sample.
    double vb1 = 305.7, vb2 = 322.1;

    // V1A -> rhythm pre-gain network -> V1B. CLEAN and BLUE each have their own copy (they differ in the R23 / C53
    // shunt), so a switch between them can crossfade like any other channel switch.
    struct RhythmFront
    {
        TriodeStage v1a, v1b;
        LinearNetwork<5> net;
        int top = 0, bot = 0;
    };
    RhythmFront clean, crunch; // CLEAN, BLUE
    TriodeStage v2a;           // BLUE
    LateStages5150 late;       // BLUE: V2B -> V5B -> V5A, solved together
    LeadPreamp5150 lead;       // RED: the whole preamp (V1A .. V5A with all networks) solved as one circuit

    // diagnostics: the lead preamp also computes every cathode and plate voltage (lead.getVk / getVp, tests)
    explicit Preamp5150 (bool diagnostics = false) : lead (diagnostics)
    {
        // ---- rhythm pre-gain network: C6 -> R19||C51 -> [R23+C53 clean shunt] -> VR1 (+C27) ----
        for (auto* f : { &clean, &crunch })
        {
            auto& net = f->net;
            net.addR (SRC, 4, kRoutV1A); // V1A plate output impedance
            net.addC (4, 0, 22e-9);      // C6
            net.addR (0, 1, 470e3);      // R19
            net.addC (0, 1, 270e-12);    // C51
            net.addR (1, 3, f == &clean ? 39e3 : 1e12); // R23 (switched out for crunch)
            net.addC (3, GND, 0.1e-6);   // C53
            f->top = net.addR (1, 2, 5e5); // VR1 upper
            f->bot = net.addR (2, GND, 5e5); // VR1 lower
            net.addC (1, 2, 180e-12);    // C27 bright
        }

        // ---- tone stack and its feed as on the schematic (factory sheet 1 and the redraw agree):
        // nodes 0 trebTop, 1 out, 2 trebBot / bassTop, 3 x, 4 bassBot / midTop, 5 C59-R94, 6 in, 7 mid wiper.
        // The driving plate feeds the stack through R89 470k into R24 33k and C59 + R94 to ground (~-24 dB).
        tsSrc = toneStack.addR (SRC, 6, kR89 + kRoutV5Afb); // R89 + driving plate's output impedance
        toneStack.addR (6, GND, 33e3);    // R24
        toneStack.addC (6, 5, 10e-9);     // C59
        toneStack.addR (5, GND, 47e3);    // R94
        toneStack.addC (6, 0, 570e-12);   // C14 470p || C60 100p (treble cap)
        tsTrebUp = toneStack.addR (0, 1, 125e3); // VR5 treble 250k upper
        tsTrebLo = toneStack.addR (1, 2, 125e3); // VR5 lower
        toneStack.addR (6, 3, 47e3);      // R28 slope
        toneStack.addC (3, 2, 22e-9);     // C18 bass cap
        toneStack.addC (3, 7, 22e-9);     // C21 mid cap -> VR4 wiper
        tsBass = toneStack.addR (2, 4, 5e5);    // VR3 bass 1M (rheostat)
        tsMidUp = toneStack.addR (4, 7, 25e3);  // VR4 mid 50k, wiper to top (bass end)
        tsMidLo = toneStack.addR (7, GND, 25e3); // VR4 wiper to ground
        toneStack.addR (4, GND, 47e3);    // R95
        toneStack.addR (1, GND, 5e5);     // load: VR7 Ultra Post 1M || VR6 Clean Post 1M
    }

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        clean.net.prepare (fs);
        crunch.net.prepare (fs);
        toneStack.prepare (fs);
        configureStages();
        late.configure (vb2, kRoutV2A, fs);
        lead.configure (vb1, vb2, fs);
        // idle draw of the stages CLEAN bypasses: in the real amp every tube is lit and draws from its rail
        idleI1V2A = dcPlateCurrent (vb1, 100e3, 39e3);
        idleI2Late = dcPlateCurrent (vb2, 220e3, 1.82e3) + dcPlateCurrent (vb2, 220e3, 2.2e3) + dcPlateCurrent (vb2, 100e3, 2.2e3);
        fadeLen = std::max (1, (int) (switchFadeMs * 1e-3 * fs));
        fadePos = fadeLen; // no switch in progress
        fadeW = 1.0;
        std::fill (std::begin (rampPos), std::end (rampPos), fadeLen);
        prevMode = mode;
        toneStack.setR (tsSrc, sourceR (mode));

        // let coupling / cathode caps settle
        for (int i = 0; i < (int) (fs * 0.3); ++i)
            process (0.0);
        fresh = true;
    }

    // Channel switch. The channels are separate circuits here (in the amp, relays rewire one set of tubes), so a
    // hard switch is a step: the tone stack, still charged by the old channel's large plate swing, suddenly sees
    // the new one, and the new circuit starts with the input mid-waveform (a step through its coupling caps).
    // Instead, for switchFadeMs both channels run: the new one from its idle point with its input ramped in, and
    // the two are crossfaded (each with its channel trim) before the tone stack, whose source impedance follows.
    void setMode (int newMode)
    {
        if (newMode == mode) return;
        if (fresh) { setModeNow (newMode); return; } // no audio since prepare(): nothing to fade from
        if (fadePos < fadeLen && newMode == prevMode)
        {
            // switched back mid-fade: reverse it (the weights continue smoothly, nothing restarts)
            std::swap (mode, prevMode);
            fadePos = fadeLen - fadePos;
            return;
        }
        if (fadePos < fadeLen && fadeW > 0.5) prevMode = mode; // a third channel mid-fade: the louder one fades out
        else if (fadePos >= fadeLen) prevMode = mode;
        mode = newMode;
        resetPath (mode);
        fadePos = 0;
    }

    // Immediate switch without a crossfade, the channel's circuits from their idle point: for when nothing is
    // audible yet (the first block after prepare)
    void setModeNow (int newMode)
    {
        if (newMode == mode && fadePos >= fadeLen) return;
        mode = prevMode = newMode;
        resetPath (mode);
        rampPos[mode] = fadeLen;
        fadePos = fadeLen;
        fadeW = 1.0;
        toneStack.setR (tsSrc, sourceR (mode));
    }

    // per-channel gain at the preamp output (AmpSim's Post trim): inside the crossfade, so a switch between
    // channels of very different trim (CLEAN 25x) does not swell the old channel mid-fade
    void setChannelTrims (double clean, double blue, double red) { trim[Clean] = clean; trim[Blue] = blue; trim[Red] = red; }

    // channel-switch crossfade length (takes effect at the next prepare)
    double switchFadeMs = 20.0;

    // all 0..1 knob positions
    void setControls (double preGain, double bass, double mid, double treble, double post)
    {
        preGain = std::clamp (preGain, 0.0, 1.0);
        setPreGainPots (leadPreTaper (preGain), taperPot (preGain));
        setToneStackRaw (std::clamp (treble, 0.0, 1.0), std::clamp (mid, 0.0, 1.0),
                         taperA (std::clamp (bass, 0.0, 1.0)));
        post = std::clamp (post, 0.0, 1.0);
        postGain = masterTaper (post);
    }

    // raw pot fractions (treble wiper from bottom, mid / bass rheostat fraction)
    void setToneStackRaw (double t, double m, double l)
    {
        toneStack.setR (tsTrebUp, (1.0 - t) * 250e3);
        toneStack.setR (tsTrebLo, t * 250e3);
        toneStack.setR (tsBass, l * 1e6);
        toneStack.setR (tsMidUp, (1.0 - m) * 50e3);
        toneStack.setR (tsMidLo, m * 50e3);
    }

    double processToneStack (double x) { return toneStack.process (x, 1); }

    // raw pot fraction (wiper position from ground), used by the tests
    void setPreGainPot (double f) { setPreGainPots (f, f); }

    // lead (VR2 Ultra Pre) and rhythm (VR1 Clean Pre) wiper fractions
    void setPreGainPots (double leadPot, double rhythm)
    {
        rhythm = std::clamp (rhythm, 0.0, 1.0);
        lead.setGainPot (leadPot); // clamps
        for (auto* f : { &clean, &crunch })
        {
            f->net.setR (f->top, (1.0 - rhythm) * 1e6);
            f->net.setR (f->bot, rhythm * 1e6);
        }
    }

    // input: guitar voltage. output: volts after tone stack and post-gain pot (channel trim included)
    double process (double vin)
    {
        fresh = false;
        double x;
        if (fadePos < fadeLen)
        {
            ++fadePos;
            fadeW = 0.5 - 0.5 * std::cos (kPi * (double) fadePos / (double) fadeLen); // 0 -> 1
            if (fadePos % 32 == 1 || fadePos == fadeLen) // the tone stack's source impedance slides with the fade
                toneStack.setR (tsSrc, sourceR (prevMode) + fadeW * (sourceR (mode) - sourceR (prevMode)));
            const double xOld = runPath (prevMode, vin) * trim[prevMode];
            const double xNew = runPath (mode, vin) * trim[mode];
            x = (1.0 - fadeW) * xOld + fadeW * xNew;
        }
        else
            x = runPath (mode, vin) * trim[mode];

        // Tone stack through R89 / R24 as on the schematic. Against the NAM captures it is ~3 dB brighter above
        // 5 kHz and ~2.5 dB lower in the mids than v0.4's fitted stand-in (a 60 pF low-pass, no R24 network); no
        // unknown part (load, OT capacitance) explains that cleanly and the captures include a load box's line
        // out, so the circuit stays as drawn (docs/nam_calibration.md).
        lastPreampOut = x;
        return toneStack.process (x, 1) * postGain;
    }

    // tone-regression reference renders (see TriodeStage::setReferencePrecision)
    void setReferencePrecision (bool on)
    {
        for (auto* s : { &clean.v1a, &clean.v1b, &crunch.v1a, &crunch.v1b, &v2a })
            s->setReferencePrecision (on);
        late.setReferencePrecision (on);
        lead.setReferencePrecision (on);
    }

    // Supply rails V1+ / V2+ for the next sample (from PowerAmp5150's supply chain)
    void setRails (double v1, double v2)
    {
        lead.setRails (v1, v2);
        for (auto* s : { &clean.v1a, &clean.v1b, &crunch.v1a, &crunch.v1b, &v2a })
            s->setSupply (v1);
        late.setSupply (v2);
    }

    // current drawn from V1+ / V2+ at the last sample, by the channel's active stages (the bypassed ones idle);
    // during a switch the two channels' draws are blended with the crossfade
    void railCurrents (double& i1, double& i2) const
    {
        pathCurrents (mode, i1, i2);
        if (fadePos < fadeLen)
        {
            double o1, o2;
            pathCurrents (prevMode, o1, o2);
            i1 = (1.0 - fadeW) * o1 + fadeW * i1;
            i2 = (1.0 - fadeW) * o2 + fadeW * i2;
        }
    }

    double getLastPreampOut() const { return lastPreampOut; }
    int getMode() const { return mode; }

private:
    static constexpr double kPi = 3.14159265358979323846;

    // one sample of a channel's preamp up to the tone stack input. A channel started from its idle point
    // (resetPath) gets its input ramped in over switchFadeMs, so it never sees a step.
    double runPath (int m, double vin)
    {
        if (rampPos[m] < fadeLen)
            vin *= 0.5 - 0.5 * std::cos (kPi * (double) ++rampPos[m] / (double) fadeLen);
        if (m == Red) return lead.process (vin);
        auto& f = m == Clean ? clean : crunch;
        const double x = f.v1b.process (f.net.process (f.v1a.process (vin), 2));
        return m == Blue ? late.process (v2a.process (x)) : x;
    }

    // a channel's circuits back to their idle point (it starts the fade-in from there)
    void resetPath (int m)
    {
        rampPos[m] = 0;
        if (m == Red) { lead.reset(); return; }
        auto& f = m == Clean ? clean : crunch;
        f.v1a.reset(); f.v1b.reset(); f.net.reset();
        if (m == Blue) { v2a.reset(); late.reset(); }
    }

    void pathCurrents (int m, double& i1, double& i2) const
    {
        if (m == Red) { i1 = lead.railCurrent1(); i2 = lead.railCurrent2(); return; }
        const auto& f = m == Clean ? clean : crunch;
        i1 = f.v1a.getPlateCurrent() + f.v1b.getPlateCurrent() + (m == Blue ? v2a.getPlateCurrent() : idleI1V2A);
        i2 = m == Blue ? late.railCurrent() : idleI2Late;
    }

    // R89 + the output impedance of the plate that drives it in channel m
    static double sourceR (int m) { return kR89 + (m == Clean ? kRoutV1B : kRoutV5Afb); }

    // idle (grid at 0 V) plate current of a cathode-biased stage
    static double dcPlateCurrent (double vb, double rp, double rk)
    {
        TriodeConfig c; c.vb = vb; c.rp = rp; c.rk = rk;
        TriodeStage s; s.configure (c, 352800.0);
        return s.getPlateCurrent();
    }

    // the CLEAN / BLUE stages (rhythm relay positions; RED runs `lead` instead)
    void configureStages()
    {
        TriodeConfig c;

        // V1A: input jack R27 22k -> C15 0.1u -> R25 1M -> R22 68k
        c = {}; c.vb = vb1; c.rsrc = 22e3; c.cc = 0.1e-6; c.ra = 1e6; c.rs = 68e3;
        c.rp = 220e3; c.rk = 1.82e3; c.ck = 1e-6;
        clean.v1a.configure (c, fs);
        crunch.v1a.configure (c, fs);

        // V1B: R82 470k grid feed; C26 22u via R18 47k (rhythm)
        c = {}; c.vb = vb1; c.rsrc = 470e3; c.rs = 1e3;
        c.rp = 100e3; c.rk = 1.82e3; c.ck = 1e-6; c.ck2 = 22e-6; c.rk2 = 47e3;
        clean.v1b.configure (c, fs);
        crunch.v1b.configure (c, fs);

        // V2A: C2 22n -> R6 470k -> grid, R9 1M || R20 220k (rhythm); 39k cold cathode
        c = {}; c.vb = vb1; c.rsrc = kRoutV1B; c.cc = 22e-9; c.rs = 470e3; c.rb = (1e6 * 220e3) / (1e6 + 220e3);
        c.rp = 100e3; c.rk = 39e3;
        v2a.configure (c, fs);
        // V2B, V5B, V5A: LateStages5150 (configured in prepare)
    }

    // Thevenin output impedance of each driving plate: Rp || (rp + (mu+1) Rk_unbypassed), rp ~ 62k
    static constexpr double kRoutV1A = 220e3 * 62e3 / (220e3 + 62e3);
    static constexpr double kRoutV1B = 100e3 * 62e3 / (100e3 + 62e3);
    static constexpr double kRoutV2A = 100e3 * (62e3 + 101 * 39e3) / (100e3 + 62e3 + 101 * 39e3);
    static constexpr double kRoutV5A = 100e3 * (62e3 + 101 * 2.2e3) / (100e3 + 62e3 + 101 * 2.2e3);
    static constexpr double kR89 = 470e3;
    // V5A with R88 shunt feedback: output impedance ~ kRoutV5A / (1 + loop gain ~9) ~ 7 kOhm (open-loop gain
    // mu Rp / (rp + Rp + (mu+1) Rk) ~ 26, feedback fraction ~ 0.35). Small next to R89, so not critical.
    static constexpr double kRoutV5Afb = kRoutV5A / (1.0 + 26.0 * 0.346);

    double fs = 192000.0;
    int mode = Red, prevMode = Red;
    int fadeLen = 1, fadePos = 1; // channel-switch crossfade (fadePos >= fadeLen: none in progress)
    double fadeW = 1.0;           // weight of `mode` (prevMode has 1 - fadeW)
    int rampPos[3] { 1, 1, 1 };   // per channel: samples of input ramp-in done (>= fadeLen: full input)
    bool fresh = false;           // prepared, no audio yet: setMode() switches at once
    double trim[3] { 1.0, 1.0, 1.0 };
    double postGain = 0.1;
    double lastPreampOut = 0.0;
    double idleI1V2A = 0.0, idleI2Late = 0.0; // CLEAN: rail draw of the bypassed stages

    LinearNetwork<8> toneStack;
    int tsSrc = 0, tsTrebUp = 0, tsTrebLo = 0, tsBass = 0, tsMidUp = 0, tsMidLo = 0;
};

} // namespace vamp_baseline
