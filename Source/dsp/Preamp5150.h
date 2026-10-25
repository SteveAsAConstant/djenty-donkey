#pragma once
#include <algorithm>
#include <cmath>
#include "LinearNetwork.h"
#include "FullPreamp5150.h"
#include "PotTapers.h"
#include "TubeSet.h"
#include "TriodeStage.h"

namespace vamp
{
// Pre-gain and post-gain (Master) pots: taperPot (PotTapers.h), the knobs read like the real amp's dials
constexpr double kPostTrimV04 = 0.229; // v0.4's Master scale: keeps old presets on the same knob positions
inline double masterTaper (double x) { return x <= 0.0 ? 0.0 : taperPot (x); }
inline double masterKnobForDrive (double drive) { return taperPotInverse (std::clamp (drive, 0.0, 1.0)); }

// Lead pre-gain: taperPot lifted. Pre-gain fits (power-amp drive fixed at the Master calibration,
// nam_compare --fit-pre) to all 14 captures want +9.3 / +6.8 / +7.2 / +4.0 dB re taperPot at dials 2 / 3 / 4 / 5
// (means of 4 / 5 / 3 / 2 captures, spread +-0.5..0.8 dB; +-1.3 at 5); interpolated linearly in dB between them,
// held below 2. v0.5.0-beta's curve was ~4 dB lower at every dial: it was fitted with C26 straight to ground
// in LEAD and without the rhythm network on C6; with R18 in series (factory schematic, FullPreamp5150::r18,
// -3.3 dB at the 110 Hz of the fits) and the rhythm network loading C6 (-0.6 dB) the preamp has that much less
// gain, a uniform offset (harmonics and compression match as before). Nothing above 5 was captured: the lift falls
// linearly to 0 at dial 10, so dial 10 is still the pot fully open (and the curve stays monotonic: taperPot grows
// ~6 dB per dial step there). Lead only: these are the lead captures.
inline double leadPreLiftDb (double x)
{
    constexpr double at[4] = { 9.32, 6.84, 7.20, 3.96 }; // dials 2, 3, 4, 5
    if (x >= 0.5) return at[3] * std::max (0.0, (1.0 - x) / 0.5);
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

// Rhythm pre-gain (VR1 Clean Pre; BLUE and CLEAN): taperPot lifted. Fits to the 22 straight-in Rhythm captures of
// the same 6505 (all with crunch on: crunch off misses them by 16-19 dB of gain and 2.9 dB of compression) want
// +3.0 / +7.1 / +7.8 dB re taperPot at dials 5 / 6 / 7 (4 / 6 / 5 captures, spread +-0.8..1.4 dB; one dial-6
// capture at -2.7 left out); at dial 8 the cost is flat (fully saturated), so nothing is read from it. Held below 5
// (not captured); above 7 the lift falls linearly to 0 at dial 10 (pot fully open), as the lead's. Monotonic:
// taperPot grows 5.4-5.5 dB per dial step in this range.
inline double rhythmPreLiftDb (double x)
{
    constexpr double at[3] = { 3.04, 7.10, 7.81 }; // dials 5, 6, 7
    if (x >= 0.7) return at[2] * std::max (0.0, (1.0 - x) / 0.3);
    if (x <= 0.5) return at[0];
    const double f = (x - 0.5) * 10.0;
    const int i = std::min (1, (int) f);
    return at[i] + (f - i) * (at[i + 1] - at[i]);
}
inline double rhythmPreTaper (double x)
{
    return std::min (1.0, taperPot (x) * std::pow (10.0, rhythmPreLiftDb (x) / 20.0));
}
inline double rhythmPreTaperInverse (double f) // monotonic: bisection (not used on the audio thread)
{
    double lo = 0.0, hi = 1.0;
    for (int i = 0; i < 60; ++i) { const double mid = 0.5 * (lo + hi); (rhythmPreTaper (mid) < f ? lo : hi) = mid; }
    return 0.5 * (lo + hi);
}

// v0.3 knob curves (pre-gain taperA, Master 60 dB log), to convert old presets / sessions
inline double masterTaperV03 (double x) { return x <= 0.0 ? 0.0 : std::pow (10.0, 3.0 * (x - 1.0)) * std::min (1.0, x * 20.0); }
inline double gainKnobFromV03 (double x, bool lead) { return lead ? leadPreTaperInverse (taperA (x)) : rhythmPreTaperInverse (taperA (x)); }
// (v0.3 -> v0.4 knob positions, which v0.5 keeps)
inline double masterKnobFromV03 (double x) { return taperPotInverse (std::clamp (masterTaperV03 (x) / kPostTrimV04, 0.0, 1.0)); }

// Component-level model of the EVH 5150 / 6505 preamp + tone stack. Each channel is its circuit with the
// channel's relay positions (factory schematic sheet 1; docs/5150_schematic_notes.md), solved as one circuit:
//  RED   (5150 lead):     FullPreamp5150 (Lead):   V1A -> both gain nets, VR2 -> V1B (R18) -> V2A -> V2B -> V5B -> V5A
//  BLUE  (6505 crunch):   FullPreamp5150 (Crunch): V1A -> both gain nets, VR1 -> V1B -> V2A (R20) -> ... -> V5A
//        CRUNCH off: FullPreamp5150 (Clean): the rhythm network at rest (K4), the same five stages (the amp's)
//  CLEAN (6505 rhythm, optimized): CleanPreamp5150: V1A -> both gain nets, VR1 -> V1B
//         "optimized" = V2A..V5A bypassed for headroom (not in the original amp)
// then the tone stack.
class Preamp5150
{
public:
    enum Mode { Clean = 0, Blue = 1, Red = 2 };

    // idle preamp B+ (V1+, V2+): the DC solution of the factory supply chain (PowerAmp5150, schematic sheet 2) with
    // the model's tube currents. With the supply connected (setRails) they follow the sagging rails per sample.
    double vb1 = 305.7, vb2 = 322.1;

    FullPreamp5150 lead;    // RED
    FullPreamp5150 blue;    // BLUE (crunch)
    CleanPreamp5150 clean;  // CLEAN
    FullPreamp5150 blueOff; // BLUE with CRUNCH off

    // diagnostics: RED and BLUE also compute every cathode and plate voltage (getVk / getVp, tests).
    // blueCrunch: the CRUNCH switch's initial position (false: BLUE starts as the rhythm channel with crunch off)
    explicit Preamp5150 (bool diagnostics = false, bool blueCrunch = true)
        : lead (channel5150::Lead, diagnostics), blue (channel5150::Crunch, diagnostics),
          blueOff (channel5150::Clean, diagnostics), crunch (blueCrunch)
    {
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
        toneStack.prepare (fs);
        setBright (bright);
        lead.configure (vb1, vb2, fs);
        blue.configure (vb1, vb2, fs);
        clean.configure (vb1, fs);
        blueOff.configure (vb1, vb2, fs);
        // idle draw of the stages CLEAN bypasses: in the real amp every tube is lit and draws from its rail
        idleI1V2A = dcPlateCurrent (vb1, 100e3, 39e3);
        idleI2Late = dcPlateCurrent (vb2, 220e3, 1.82e3) + dcPlateCurrent (vb2, 220e3, 2.2e3) + dcPlateCurrent (vb2, 100e3, 2.2e3);
        fadeLen = std::max (1, (int) (switchFadeMs * 1e-3 * fs));
        fadePos = fadeLen; // no switch in progress
        path = pathOf (mode);
        for (int c = 0; c < kPaths; ++c) w[c] = w0[c] = c == path ? 1.0 : 0.0;
        toneStack.setR (tsSrc, sourceR (path));

        // let coupling / cathode caps settle
        for (int i = 0; i < (int) (fs * 0.3); ++i)
            process (0.0);
        fresh = true;
    }

    // Channel switch. The channels are separate circuits here; in the amp, relays rewire one set of tubes and every
    // capacitor keeps its charge. So the new channel's circuit takes over the old one's state wherever the part is
    // the same (warmFrom: tubes, coupling and cathode capacitors; started from idle instead, a high-gain channel
    // first has to build up its bias, a transient). The rewired networks themselves still change at once, and the
    // tone stack's source with them, so for switchFadeMs both channels run and are crossfaded (each with its channel
    // trim) before the tone stack, whose source impedance follows.
    // Every channel has a weight (summing to 1): a switch starts from the weights of the moment, also mid-fade (back
    // to the previous channel, or on to the third one), and moves them to the new channel over switchFadeMs, so no
    // weight ever jumps. A channel runs while its weight is above 0; one coming in from silence takes over the
    // state of the loudest running channel.
    void setMode (int newMode)
    {
        if (newMode == mode) return;
        mode = newMode;
        switchPath (pathOf (mode));
    }

    // CRUNCH (rhythm, relay K4): BLUE's gain network. Switched like a channel (K4 rewires the same tubes).
    void setCrunch (bool on)
    {
        if (on == crunch) return;
        crunch = on;
        switchPath (pathOf (mode));
    }
    bool getCrunch() const { return crunch; }

    // Immediate switch without a crossfade, the channel's circuits from their idle point: for when nothing is
    // audible yet (the first block after prepare)
    void setModeNow (int newMode)
    {
        if (newMode == mode && fadePos >= fadeLen) return;
        mode = newMode;
        path = pathOf (mode);
        resetPath (path);
        fadePos = fadeLen;
        for (int c = 0; c < kPaths; ++c) w[c] = w0[c] = c == path ? 1.0 : 0.0;
        toneStack.setR (tsSrc, sourceR (path));
    }

    // per-channel gain at the preamp output (AmpSim's Post trim): inside the crossfade, so a switch between
    // channels of very different trim (CLEAN 25x) does not swell the old channel mid-fade. BLUE's trim for both
    // CRUNCH positions (the same Post pot).
    void setChannelTrims (double cleanTrim, double blueTrim, double redTrim)
    {
        trim[PClean] = cleanTrim; trim[PBlue] = trim[PBlueOff] = blueTrim; trim[PRed] = redTrim;
    }

    // BRIGHT (rhythm) is on by default: BLUE was calibrated with it (docs/nam_calibration.md)
    bool bright = true;

    // channel-switch crossfade length (takes effect at the next prepare)
    double switchFadeMs = 20.0;

    // all 0..1 knob positions
    void setControls (double preGain, double bass, double mid, double treble, double post)
    {
        preGain = std::clamp (preGain, 0.0, 1.0);
        setPreGainPots (leadPreTaper (preGain), rhythmPreTaper (preGain));
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

    // lead (VR2 Ultra Pre) and rhythm (VR1 Clean Pre) wiper fractions: both pots are in every channel's circuit
    void setPreGainPots (double leadPot, double rhythm)
    {
        lead.setGainPots (leadPot, rhythm);
        blue.setGainPots (leadPot, rhythm);
        clean.setGainPots (leadPot, rhythm);
        blueOff.setGainPots (leadPot, rhythm);
    }

    // BRIGHT pushbutton (S1C shorts R75: C10 across R21). In every channel's circuit; audible in CLEAN and BLUE.
    void setBright (bool on)
    {
        bright = on;
        lead.setBright (on);
        blue.setBright (on);
        clean.setBright (on);
        blueOff.setBright (on);
    }

    // Tube set (TubeSet): V1A V1B V2A V2B V5B V5A as particular tubes. The same tube in every channel's circuit (the
    // relays only rewire around it); CLEAN uses V1A and V1B. Takes effect at once, as swapping tubes.
    void setTubeSet (const TubeSet& s)
    {
        for (int t = 0; t < 6; ++t)
        {
            lead.net.setTubeVariation (t, s.preI[t], s.preMu[t]);
            blue.net.setTubeVariation (t, s.preI[t], s.preMu[t]);
            blueOff.net.setTubeVariation (t, s.preI[t], s.preMu[t]);
            if (t < 2) clean.net.setTubeVariation (t, s.preI[t], s.preMu[t]);
        }
    }

    // Circuit noise: every resistor in the grid networks (thermal) and every tube (shot, flicker) of the three channel
    // circuits, independent per circuit (TubeNoise, DKCircuit::setNoise). Off by default.
    void setNoise (bool on)
    {
        if (on == noise) return;
        noise = on;
        lead.setNoise (on, 0x1234567ull);
        blue.setNoise (on, 0x89abcdefull);
        clean.setNoise (on, 0x51ed270bull);
        blueOff.setNoise (on, 0x2f6b9c4dull);
    }
    bool getNoise() const { return noise; }

    // input: guitar voltage. output: volts after tone stack and post-gain pot (channel trim included)
    double process (double vin)
    {
        fresh = false;
        double x;
        if (fadePos < fadeLen)
        {
            ++fadePos;
            const double s = 0.5 - 0.5 * std::cos (kPi * (double) fadePos / (double) fadeLen); // 0 -> 1
            x = 0.0;
            double rs = 0.0;
            for (int c = 0; c < kPaths; ++c)
            {
                const bool running = w[c] > 0.0 || c == path;
                w[c] = fadePos >= fadeLen ? (c == path ? 1.0 : 0.0) : w0[c] * (1.0 - s) + (c == path ? s : 0.0);
                if (running) x += w[c] * runPath (c, vin) * trim[c]; // a fading-out channel runs to the end
                rs += w[c] * sourceR (c);
            }
            if (fadePos % 32 == 1 || fadePos == fadeLen) // the tone stack's source impedance slides with the fade
                toneStack.setR (tsSrc, rs);
        }
        else
            x = runPath (path, vin) * trim[path];

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
        lead.setReferencePrecision (on);
        blue.setReferencePrecision (on);
        clean.setReferencePrecision (on);
        blueOff.setReferencePrecision (on);
    }

    // Supply rails V1+ / V2+ for the next sample (from PowerAmp5150's supply chain)
    void setRails (double v1, double v2)
    {
        lead.setRails (v1, v2);
        blue.setRails (v1, v2);
        clean.setRails (v1);
        blueOff.setRails (v1, v2);
    }

    // current drawn from V1+ / V2+ at the last sample, by the channel's active stages (the bypassed ones idle);
    // during a switch the two channels' draws are blended with the crossfade
    void railCurrents (double& i1, double& i2) const
    {
        pathCurrents (path, i1, i2);
        if (fadePos < fadeLen)
        {
            i1 = i2 = 0.0;
            for (int c = 0; c < kPaths; ++c)
                if (w[c] > 0.0)
                {
                    double o1, o2;
                    pathCurrents (c, o1, o2);
                    i1 += w[c] * o1;
                    i2 += w[c] * o2;
                }
        }
    }

    double getLastPreampOut() const { return lastPreampOut; }
    int getMode() const { return mode; }

private:
    static constexpr double kPi = 3.14159265358979323846;

    // the circuits ("paths"): a channel, and for BLUE the CRUNCH position
    enum Path { PClean = 0, PBlue = 1, PRed = 2, PBlueOff = 3 };
    static constexpr int kPaths = 4;
    int pathOf (int channel) const { return channel == Blue && ! crunch ? PBlueOff : channel; }

    // to circuit p, crossfaded from whatever runs now (see setMode)
    void switchPath (int p)
    {
        if (p == path) return;
        if (fresh) { path = p; resetPath (p); for (int c = 0; c < kPaths; ++c) w[c] = w0[c] = c == p ? 1.0 : 0.0;
                     toneStack.setR (tsSrc, sourceR (p)); return; } // no audio since prepare(): nothing to fade from
        if (w[p] <= 0.0)
        {
            int from = path;
            for (int c = 0; c < kPaths; ++c) if (c != p && w[c] > w[from]) from = c;
            enterPath (p, from);
        }
        std::copy (std::begin (w), std::end (w), std::begin (w0));
        path = p;
        fadePos = 0;
    }

    FullPreamp5150& full (int p) { return p == PRed ? lead : p == PBlue ? blue : blueOff; }
    const FullPreamp5150& full (int p) const { return p == PRed ? lead : p == PBlue ? blue : blueOff; }

    // one sample of a circuit up to the tone stack input
    double runPath (int p, double vin) { return p == PClean ? clean.process (vin) : full (p).process (vin); }

    // a circuit back to its idle point
    void resetPath (int p)
    {
        if (p == PClean) clean.reset();
        else full (p).reset();
    }

    // circuit p comes in: it takes over the state of circuit `from` (see setMode)
    void enterPath (int p, int from)
    {
        if (p == PClean) { clean.warmFrom (full (from)); return; }
        if (from == PClean) full (p).warmFrom (clean);
        else full (p).warmFrom (full (from));
    }

    void pathCurrents (int p, double& i1, double& i2) const
    {
        if (p != PClean)
        {
            i1 = full (p).railCurrent1();
            i2 = full (p).railCurrent2();
            return;
        }
        i1 = clean.railCurrent1() + idleI1V2A;
        i2 = idleI2Late;
    }

    // R89 + the output impedance of the plate that drives it in circuit p
    static double sourceR (int p) { return kR89 + (p == PClean ? kRoutV1B : kRoutV5Afb); }

    // idle (grid at 0 V) plate current of a cathode-biased stage
    static double dcPlateCurrent (double vb, double rp, double rk)
    {
        TriodeConfig c; c.vb = vb; c.rp = rp; c.rk = rk;
        TriodeStage s; s.configure (c, 352800.0);
        return s.getPlateCurrent();
    }

    // Thevenin output impedance of each driving plate: Rp || (rp + (mu+1) Rk_unbypassed), rp ~ 62k
    static constexpr double kRoutV1B = 100e3 * 62e3 / (100e3 + 62e3);
    static constexpr double kRoutV5A = 100e3 * (62e3 + 101 * 2.2e3) / (100e3 + 62e3 + 101 * 2.2e3);
    static constexpr double kR89 = 470e3;
    // V5A with R88 shunt feedback: output impedance ~ kRoutV5A / (1 + loop gain ~9) ~ 7 kOhm (open-loop gain
    // mu Rp / (rp + Rp + (mu+1) Rk) ~ 26, feedback fraction ~ 0.35). Small next to R89, so not critical.
    static constexpr double kRoutV5Afb = kRoutV5A / (1.0 + 26.0 * 0.346);

    double fs = 192000.0;
    int mode = Red;               // the channel
    bool crunch = true;           // the CRUNCH switch (BLUE)
    int path = PRed;              // the circuit that runs (fading in, during a switch)
    int fadeLen = 1, fadePos = 1; // channel-switch crossfade (fadePos >= fadeLen: none in progress)
    double w[kPaths] { 0.0, 0.0, 1.0, 0.0 }, w0[kPaths] { 0.0, 0.0, 1.0, 0.0 }; // circuit weights now / at the fade start
    bool fresh = false;           // prepared, no audio yet: setMode() switches at once
    double trim[kPaths] { 1.0, 1.0, 1.0, 1.0 };
    double postGain = 0.1;
    double lastPreampOut = 0.0;
    double idleI1V2A = 0.0, idleI2Late = 0.0; // CLEAN: rail draw of the bypassed stages
    bool noise = false;

    LinearNetwork<8> toneStack;
    int tsSrc = 0, tsTrebUp = 0, tsTrebLo = 0, tsBass = 0, tsMidUp = 0, tsMidLo = 0;
};

} // namespace vamp
