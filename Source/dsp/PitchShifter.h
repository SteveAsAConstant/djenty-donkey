#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>
#include "FFT.h"
#include "Filters.h"

namespace vamp
{
// ===========================================================================
// LIVE mode: WSOLA-style delay-line pitch shifter.
//
// A read tap moves through a delay line at `ratio` samples per output sample, so its delay
// drifts. When it reaches the end of its range it is replaced by a new tap, crossfaded in over
// a few ms. The new tap's delay is not fixed: it is chosen by normalised cross-correlation so
// the two taps are waveform-aligned at the splice -> no "warble" on single notes, much less on
// chords than a blind crossfade. Latency ~ 15 ms (varies slightly with the splice cycle).
// ===========================================================================
class WsolaShifter
{
public:
    void prepare (double sampleRate)
    {
        fs = sampleRate;
        corrLen = (int) (0.005 * fs);   // correlation window
        sweep = (int) (0.020 * fs);     // delay range of one tap
        search = (int) (0.016 * fs);    // search range (>= one period of ~60 Hz)
        xfade = (int) (0.008 * fs);     // crossfade length
        envDecay = (float) std::exp (-1.0 / (0.050 * fs));
        int size = 1;
        while (size < sweep + search + 4 * xfade + corrLen + 64) size <<= 1;
        buf.assign ((size_t) size, 0.0f);
        mask = size - 1;
        setSemitones (semis, true);
        reset();
    }

    void reset()
    {
        std::fill (buf.begin(), buf.end(), 0.0f);
        w = 0;
        dA = (double) dMin;
        fading = false;
    }

    void setSemitones (int s, bool force = false)
    {
        if (s == semis && ! force) return;
        semis = s;
        ratio = std::pow (2.0, s / 12.0);
        step = 1.0 - ratio;
        // the read tap must stay behind the write pointer during a crossfade
        dMin = corrLen + (int) std::ceil (std::max (0.0, ratio - 1.0) * xfade) + 8;
        dMax = dMin + sweep;
        dA = std::clamp (dA, (double) dMin, (double) dMax);
    }

    // average delay (samples) - used to time-align the noise-gate key
    int getAverageDelay() const { return semis == 0 ? 0 : dMin + sweep / 2; }

    void process (float* data, int n)
    {
        for (int i = 0; i < n; ++i)
        {
            buf[(size_t) w] = data[i];

            // while the input is silent, park the tap at its shortest usable delay so the next
            // pick attack comes through with minimum latency (the drift only happens in sustain)
            env = std::max (std::abs (data[i]), env * envDecay);
            if (! fading && env < 1e-3f)
                dA = step > 0.0 ? (double) dMin : 0.5 * (dMin + dMax);

            if (semis != 0)
            {
                float out = read (dA);
                if (fading)
                {
                    const float t = (float) fadePos / (float) xfade;
                    out = out * (1.0f - t) + read (dB) * t;
                }
                dA += step;
                if (fading)
                {
                    dB += step;
                    if (++fadePos >= xfade) { dA = dB; fading = false; }
                }
                else if (needsSplice())
                {
                    startSplice();
                }
                data[i] = out;
            }

            w = (w + 1) & mask;
        }
    }

private:
    bool needsSplice() const
    {
        return step > 0.0 ? dA + step * xfade > dMax   // pitch down: delay grows
                          : dA + step * xfade < dMin;  // pitch up: delay shrinks
    }

    void startSplice()
    {
        // candidate delays for the new tap
        const int lo = step > 0.0 ? dMin : dMax - search;
        const int hi = step > 0.0 ? dMin + search : dMax;
        const int refStart = w - (int) std::floor (dA);

        double best = -2.0;
        int bestD = step > 0.0 ? dMin : dMax;
        for (int d = lo; d <= hi; ++d)
        {
            const int candStart = w - d;
            double xy = 0.0, xx = 0.0, yy = 0.0;
            for (int m = 0; m < corrLen; m += 2)
            {
                const double a = buf[(size_t) ((refStart + m) & mask)];
                const double b = buf[(size_t) ((candStart + m) & mask)];
                xy += a * b; xx += a * a; yy += b * b;
            }
            const double c = xy / std::sqrt (xx * yy + 1e-20);
            if (c > best) { best = c; bestD = d; }
        }

        dB = (double) bestD + (dA - std::floor (dA)); // keep the fractional phase
        fading = true;
        fadePos = 0;
    }

    // 4-point Hermite interpolation
    float read (double delay) const
    {
        const double rp = (double) w - delay;
        const int i0 = (int) std::floor (rp);
        const float f = (float) (rp - i0);
        const float xm1 = buf[(size_t) ((i0 - 1) & mask)], x0 = buf[(size_t) (i0 & mask)];
        const float x1 = buf[(size_t) ((i0 + 1) & mask)], x2 = buf[(size_t) ((i0 + 2) & mask)];
        const float c1 = 0.5f * (x1 - xm1);
        const float c2 = xm1 - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
        const float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
        return ((c3 * f + c2) * f + c1) * f + x0;
    }

    double fs = 48000.0;
    std::vector<float> buf;
    int mask = 0, w = 0;
    int corrLen = 240, sweep = 960, search = 768, xfade = 384, dMin = 256, dMax = 1216;
    int semis = 0, fadePos = 0;
    double ratio = 1.0, step = 0.0, dA = 256.0, dB = 256.0;
    float env = 0.0f, envDecay = 0.9995f;
    bool fading = false;
};

// ===========================================================================
// HQ mode: phase-vocoder time-stretch + resampling, with identity phase locking
// (Laroche & Dolson, "Improved phase vocoder time-scale modification of audio", 1999).
//
// Each analysis frame (hop Ra) is resynthesised with hop Rs = Ra * ratio (time-stretch,
// pitch unchanged); the stretched stream is then read back at speed Rs / Ra, which restores
// the duration and multiplies every frequency by exactly Rs / Ra (quantisation < 2 cents).
// Spectral peaks carry the phase (advanced by Rs * true frequency); the bins around each
// peak are phase-locked to it, which keeps chords from sounding "phasey". At note onsets the
// output phases are reset to the input phases so pick attacks stay sharp.
// Fixed latency ~ N samples (~43 ms at 48 kHz).
// ===========================================================================
class PvShifter
{
public:
    void prepare (double sampleRate)
    {
        N = sampleRate <= 50000.0 ? 2048 : (sampleRate <= 100000.0 ? 4096 : 8192);
        Ra = N / 8;
        half = N / 2;
        fft.prepare (N);
        window.resize ((size_t) N);
        for (int k = 0; k < N; ++k)
            window[(size_t) k] = 0.5 - 0.5 * std::cos (2.0 * kPi * k / N);
        inFifo.assign ((size_t) N, 0.0);
        accum.assign ((size_t) N, 0.0);
        stretched.assign ((size_t) (8 * N), 0.0f);
        sMask = 8 * N - 1;
        spec.assign ((size_t) N, {});
        mag.assign ((size_t) half + 1, 0.0);
        magPrev.assign ((size_t) half + 1, 0.0);
        phase.assign ((size_t) half + 1, 0.0);
        lastPhase.assign ((size_t) half + 1, 0.0);
        omega.assign ((size_t) half + 1, 0.0);
        outPhase.assign ((size_t) half + 1, 0.0);
        peaks.reserve ((size_t) half);
        setSemitones (semis, true);
        reset();
    }

    void reset()
    {
        std::fill (inFifo.begin(), inFifo.end(), 0.0);
        std::fill (accum.begin(), accum.end(), 0.0);
        std::fill (stretched.begin(), stretched.end(), 0.0f);
        std::fill (lastPhase.begin(), lastPhase.end(), 0.0);
        std::fill (outPhase.begin(), outPhase.end(), 0.0);
        std::fill (magPrev.begin(), magPrev.end(), 0.0);
        fluxAvg = 0.0;
        inCount = 0;
        // read position trails the write position by one synthesis hop (+ interpolation margin)
        writePos = Rs + 4;
        readPos = 0.0;
    }

    void setSemitones (int s, bool force = false)
    {
        if (s == semis && ! force) return;
        semis = s;
        Rs = std::clamp ((int) std::lround (Ra * std::pow (2.0, s / 12.0)), Ra / 4, Ra * 2);
        speed = (double) Rs / Ra;
        // keep the reader exactly one synthesis hop (+ margin) behind the writer
        writePos = (long long) std::floor (readPos) + Rs + 4;
        // Hann analysis x Hann synthesis overlap-add gain at synthesis hop Rs: 3N / (8 Rs)
        olaScale = 1.0 / ((double) N * (3.0 * N / (8.0 * Rs)));
    }

    // input-to-output delay in samples (constant for a given ratio)
    int getLatency() const { return N + 4; }

    void process (float* data, int n)
    {
        for (int i = 0; i < n; ++i)
        {
            inFifo[(size_t) (N - Ra + inCount)] = data[i];
            if (++inCount == Ra)
            {
                inCount = 0;
                processFrame();
                std::copy (inFifo.begin() + Ra, inFifo.end(), inFifo.begin());
            }
            data[i] = readStretched (readPos);
            readPos += speed;
        }
    }

private:
    static double princarg (double p) { return p - 2.0 * kPi * std::floor ((p + kPi) / (2.0 * kPi)); }

    float readStretched (double pos) const
    {
        const long long i0 = (long long) std::floor (pos);
        const float f = (float) (pos - (double) i0);
        auto at = [this] (long long k) { return stretched[(size_t) (k & sMask)]; };
        const float xm1 = at (i0 - 1), x0 = at (i0), x1 = at (i0 + 1), x2 = at (i0 + 2);
        const float c1 = 0.5f * (x1 - xm1);
        const float c2 = xm1 - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
        const float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
        return ((c3 * f + c2) * f + c1) * f + x0;
    }

    void processFrame()
    {
        // ---- analysis ----
        for (int k = 0; k < N; ++k) spec[(size_t) k] = { inFifo[(size_t) k] * window[(size_t) k], 0.0 };
        fft.transform (spec.data(), false);

        double flux = 0.0, maxMag = 0.0, energy = 0.0;
        for (int k = 0; k <= half; ++k)
        {
            const double binW = 2.0 * kPi * k / N;
            mag[(size_t) k] = std::abs (spec[(size_t) k]);
            phase[(size_t) k] = std::arg (spec[(size_t) k]);
            const double dev = princarg (phase[(size_t) k] - lastPhase[(size_t) k] - binW * Ra);
            lastPhase[(size_t) k] = phase[(size_t) k];
            omega[(size_t) k] = binW + dev / Ra;              // true frequency (rad / sample)
            flux += std::max (0.0, mag[(size_t) k] - magPrev[(size_t) k]);
            maxMag = std::max (maxMag, mag[(size_t) k]);
            energy += mag[(size_t) k];
        }
        std::copy (mag.begin(), mag.end(), magPrev.begin());

        const bool onset = flux > 0.5 * energy && flux > 2.0 * fluxAvg;
        fluxAvg = 0.8 * fluxAvg + 0.2 * flux;

        // ---- peaks + identity phase locking ----
        peaks.clear();
        const double floorMag = maxMag * 1e-6;
        for (int k = 1; k < half; ++k)
        {
            const double m = mag[(size_t) k];
            if (m > floorMag && m > mag[(size_t) k - 1] && m >= mag[(size_t) k + 1])
                peaks.push_back (k);
        }

        if (onset || peaks.empty())
        {
            for (int k = 0; k <= half; ++k) outPhase[(size_t) k] = phase[(size_t) k];
        }
        else
        {
            for (size_t pi = 0; pi < peaks.size(); ++pi)
            {
                const int p = peaks[pi];
                const int lo = pi == 0 ? 0 : (peaks[pi - 1] + p) / 2 + 1;
                const int hi = pi + 1 == peaks.size() ? half : (p + peaks[pi + 1]) / 2;
                const double phPeak = princarg (outPhase[(size_t) p] + Rs * omega[(size_t) p]);
                for (int k = lo; k <= hi; ++k)
                    if (k != p)
                        outPhase[(size_t) k] = phPeak + (phase[(size_t) k] - phase[(size_t) p]);
                outPhase[(size_t) p] = phPeak;
            }
        }

        // ---- resynthesis with hop Rs ----
        for (int k = 0; k <= half; ++k)
            spec[(size_t) k] = std::polar (mag[(size_t) k], outPhase[(size_t) k]);
        for (int k = half + 1; k < N; ++k)
            spec[(size_t) k] = std::conj (spec[(size_t) (N - k)]);
        fft.transform (spec.data(), true);

        for (int k = 0; k < N; ++k)
            accum[(size_t) k] += spec[(size_t) k].real() * window[(size_t) k] * olaScale;

        // the first Rs samples are complete -> append to the stretched stream
        for (int k = 0; k < Rs; ++k)
            stretched[(size_t) ((writePos + k) & sMask)] = (float) accum[(size_t) k];
        writePos += Rs;
        std::copy (accum.begin() + Rs, accum.end(), accum.begin());
        std::fill (accum.end() - Rs, accum.end(), 0.0);
    }

    int N = 2048, Ra = 256, Rs = 256, half = 1024, inCount = 0, semis = 0;
    long long writePos = 0, sMask = 0;
    double speed = 1.0, readPos = 0.0, olaScale = 1.0, fluxAvg = 0.0;
    FFT fft;
    std::vector<double> window, inFifo, accum;
    std::vector<float> stretched;
    std::vector<std::complex<double>> spec;
    std::vector<double> mag, magPrev, phase, lastPhase, omega, outPhase;
    std::vector<int> peaks;
};

// ===========================================================================
class PitchShifter
{
public:
    enum Mode { Live = 0, HQ = 1 };

    void prepare (double sampleRate)
    {
        live.prepare (sampleRate);
        hq.prepare (sampleRate);
    }

    void reset() { live.reset(); hq.reset(); }

    void setParams (int semitones, int newMode)
    {
        const bool needsReset = newMode != mode || (semitones == 0) != (semis == 0);
        mode = newMode;
        semis = semitones;
        live.setSemitones (semitones);
        hq.setSemitones (semitones);
        if (needsReset)
            reset();
    }

    void process (float* data, int n)
    {
        if (semis == 0) return;
        if (mode == HQ) hq.process (data, n);
        else            live.process (data, n);
    }

    // latency to report to the host (fixed in HQ mode, ~0 reported in LIVE mode)
    int getReportedLatency() const { return semis != 0 && mode == HQ ? hq.getLatency() : 0; }
    // average delay of the shifted signal (to align the noise-gate key)
    int getAverageDelay() const { return semis == 0 ? 0 : (mode == HQ ? hq.getLatency() : live.getAverageDelay()); }

private:
    WsolaShifter live;
    PvShifter hq;
    int mode = Live, semis = 0;
};

} // namespace vamp
