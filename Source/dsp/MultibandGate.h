#pragma once
#include <algorithm>
#include <cmath>
#include "Filters.h"

namespace vamp
{
// 3-band "smart" noise gate.
//
// Detection: wideband envelope of the clean, pre-amp guitar signal (the key). It is the
// most reliable "am I playing?" signal and is not fooled by the amp's own hiss.
//
// Action: applied post-amp (where the noise is audible) on 3 Linkwitz-Riley bands that
// sum back flat. Each band has its own close threshold, release time and depth:
//   - HIGH closes earliest, fastest and deepest  -> removes the fizz / hiss
//   - LOW  closes latest, slowest and shallowest -> keeps the body of the sustain
class MultibandGate
{
public:
    static constexpr double lowXover = 300.0, highXover = 3000.0;

    struct Settings
    {
        float threshDb = -60.0f;  // key level (dBFS, after the Input knob) where the gate closes
        float releaseMs = 80.0f;  // base release, scaled per band
        float depth[3] = { 0.3f, 0.6f, 1.0f }; // 0 = no reduction, 1 = -80 dB (low, mid, high)
    };

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        using T = Biquad::Type;
        constexpr double q = 0.70710678; // LR4 = two cascaded Butterworth sections

        for (auto* f : { &lpLo[0], &lpLo[1] }) f->set (T::lowPass,  fs, lowXover, q);
        for (auto* f : { &hpLo[0], &hpLo[1] }) f->set (T::highPass, fs, lowXover, q);
        for (auto* f : { &lpHi[0], &lpHi[1], &apLp[0], &apLp[1] }) f->set (T::lowPass,  fs, highXover, q);
        for (auto* f : { &hpHi[0], &hpHi[1], &apHp[0], &apHp[1] }) f->set (T::highPass, fs, highXover, q);
        keyHP.set (T::highPass, fs, 80.0, 0.7071); // ignore hum in the key

        envRelease = (float) std::exp (-1.0 / (0.010 * fs)); // detector smoothing
        attackCoef = (float) std::exp (-1.0 / (0.0005 * fs)); // 0.5 ms opening
        holdSamples = (int) (0.030 * fs);
        reset();
    }

    void reset()
    {
        for (auto* f : { &lpLo[0], &lpLo[1], &hpLo[0], &hpLo[1], &lpHi[0], &lpHi[1], &hpHi[0], &hpHi[1],
                         &apLp[0], &apLp[1], &apHp[0], &apHp[1], &keyHP })
            f->reset();
        env = 0.0f;
        for (auto& b : bands) { b.open = false; b.hold = 0; b.gain = 1.0f; }
    }

    void process (float* audio, const float* key, int n, const Settings& s)
    {
        float closedGain[3], releaseCoef[3], thresh[3];
        for (int b = 0; b < 3; ++b)
        {
            closedGain[b] = dbToGain (-80.0f * std::clamp (s.depth[b], 0.0f, 1.0f));
            const double relSec = std::max (1.0, (double) s.releaseMs * releaseFactor[b]) * 0.001;
            releaseCoef[b] = (float) std::exp (-1.0 / (relSec * fs));
            thresh[b] = s.threshDb + threshOffsetDb[b];
        }

        for (int i = 0; i < n; ++i)
        {
            // ---- detector ----
            const float k = std::abs (keyHP.process (key[i]));
            env = k > env ? k : env * envRelease;
            const float envDb = 20.0f * std::log10 (env + 1e-9f);

            // ---- 3-band LR4 split (low band all-passed so the bands stay phase aligned) ----
            const float x = audio[i];
            const float lo = lpLo[1].process (lpLo[0].process (x));
            const float hi = hpLo[1].process (hpLo[0].process (x));
            const float band[3] = {
                apLp[1].process (apLp[0].process (lo)) + apHp[1].process (apHp[0].process (lo)),
                lpHi[1].process (lpHi[0].process (hi)),
                hpHi[1].process (hpHi[0].process (hi)),
            };

            float out = 0.0f;
            for (int b = 0; b < 3; ++b)
            {
                auto& st = bands[b];
                if (envDb > thresh[b])                      { st.open = true; st.hold = holdSamples; }
                else if (envDb < thresh[b] - hysteresisDb)  { if (st.hold > 0) --st.hold; else st.open = false; }

                const float target = st.open ? 1.0f : closedGain[b];
                const float c = target > st.gain ? attackCoef : releaseCoef[b];
                st.gain = target + (st.gain - target) * c;
                out += band[b] * st.gain;
            }
            audio[i] = out;
        }
    }

    // current gain of each band (0..1), for the UI
    float getBandGain (int b) const { return bands[b].gain; }

private:
    // per band: low, mid, high
    static constexpr float threshOffsetDb[3] = { 0.0f, 3.0f, 8.0f };
    static constexpr float releaseFactor[3] = { 2.0f, 1.0f, 0.4f };
    static constexpr float hysteresisDb = 4.0f;

    struct BandState { bool open = false; int hold = 0; float gain = 1.0f; };

    double fs = 48000.0;
    Biquad lpLo[2], hpLo[2], lpHi[2], hpHi[2], apLp[2], apHp[2], keyHP;
    float env = 0.0f, envRelease = 0.99f, attackCoef = 0.9f;
    int holdSamples = 1440;
    BandState bands[3];
};

// Fast single-band gate in front of the amp (the "input" half of a 4-cable gate setup).
// Its job is feel, not hiss: it makes the gaps between palm-muted chugs dead silent so the
// high-gain amp doesn't amplify string ringing / pickup noise between notes.
class InputGate
{
public:
    void prepare (double sampleRate)
    {
        fs = sampleRate;
        keyHP.set (Biquad::Type::highPass, fs, 80.0, 0.7071);
        envRelease = (float) std::exp (-1.0 / (0.005 * fs));  // 5 ms detector smoothing
        attackCoef = (float) std::exp (-1.0 / (0.0003 * fs)); // 0.3 ms opening
        holdSamples = (int) (0.005 * fs);
        reset();
    }

    void reset() { keyHP.reset(); env = 0.0f; open = false; hold = 0; gain = 1.0f; gainDb = 0.0f; }

    // releaseMs = time to close completely (linear fade in dB down to -80 dB)
    void process (float* audio, int n, float threshDb, float releaseMs)
    {
        const float relStepDb = 80.0f / (std::max (1.0f, releaseMs) * 0.001f * (float) fs);
        for (int i = 0; i < n; ++i)
        {
            const float k = std::abs (keyHP.process (audio[i]));
            env = k > env ? k : env * envRelease;
            const float envDb = 20.0f * std::log10 (env + 1e-9f);

            if (envDb > threshDb)                 { open = true; hold = holdSamples; }
            else if (envDb < threshDb - 4.0f)     { if (hold > 0) --hold; else open = false; }

            if (open) gain = 1.0f + (gain - 1.0f) * attackCoef;                 // 0.3 ms opening
            else      gainDb = std::max (-80.0f, gainDb - relStepDb);           // linear dB fade
            if (open) gainDb = 20.0f * std::log10 (std::max (gain, 1e-4f));
            else      gain = gainDb <= -80.0f ? 0.0f : std::pow (10.0f, gainDb / 20.0f);

            audio[i] *= gain;
        }
    }

    float getGain() const { return gain; }

private:
    double fs = 48000.0;
    Biquad keyHP;
    float env = 0.0f, envRelease = 0.99f, attackCoef = 0.9f, gain = 1.0f, gainDb = 0.0f;
    int holdSamples = 240, hold = 0;
    bool open = false;
};

} // namespace vamp
