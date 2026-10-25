// Checks of the plugin's real amp path (AmpSim at 48 kHz: JUCE polyphase-IIR oversampling, 8x) against
//   ideal  the same DSP at 4x / 8x / 16x with a long linear-phase decimator (16x: practically alias-free)
//   NAM    a capture of a real 6505 (optional), which has no aliasing of its own beyond its 48 kHz model
//
//   amp_path_test [model.nam] [input-dBu=11.5]
//
// 1. aliasing: inharmonic energy (everything that is not a harmonic of f0, 20 Hz..20 kHz) for HF sines
// 2. tone: 1/3-octave levels of the plugin path vs ideal on guitar-like input (oversampling filters)
// 3. pick attack: band levels above 2 kHz for plucks with a pick-noise transient, plugin vs NAM
// 4. dynamics: RMS envelope of a 110 Hz tone burst (quiet / loud / quiet), plugin vs NAM
// Settings: lead channel, gain 5, bass 6, mid 4, treble 6, presence 8, resonance 9, master 3
// (the "6505 1992 Ld Pr-5 B-6 M-4 T-6 Po-3 R-9 P-8" capture).
#include <array>
#include <chrono>
#include <complex>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "dsp/AmpSim.h"
#include "dsp/FFT.h"
#include "dsp/MultibandGate.h"
#include "NamWaveNet.h"

namespace
{
constexpr double kPi = vamp::kPi, fs = 48000.0;
using Signal = std::vector<double>;
using Gen = std::function<double (double)>;

Signal sample (const Gen& g, double rate, double secs)
{
    Signal x ((size_t) (rate * secs));
    for (size_t i = 0; i < x.size(); ++i) x[i] = g (i / rate);
    return x;
}

vamp::AmpSim::Settings settings()
{
    vamp::AmpSim::Settings s;
    s.channel = 2; s.gain = 0.5f; s.bass = 0.6f; s.mid = 0.4f; s.treble = 0.6f;
    s.presence = 0.8f; s.resonance = 0.9f; s.master = 0.3f;
    return s;
}

// the plugin's amp path: AmpSim at 48 kHz, 64-sample blocks (input digital = volts)
Signal pluginPath (const Gen& g, double secs)
{
    vamp::AmpSim amp;
    amp.prepare (fs, 64);
    const auto s = settings();
    std::vector<float> buf (64);
    for (int i = 0; i < 48000 / 64; ++i) { std::fill (buf.begin(), buf.end(), 0.0f); amp.process (buf.data(), 64, s); } // settle
    const auto x = sample (g, fs, secs);
    Signal y (x.size());
    for (size_t start = 0; start < x.size(); start += 64)
    {
        const int n = (int) std::min<size_t> (64, x.size() - start);
        for (int i = 0; i < n; ++i) buf[(size_t) i] = (float) x[start + (size_t) i];
        amp.process (buf.data(), n, s);
        for (int i = 0; i < n; ++i) y[start + (size_t) i] = buf[(size_t) i];
    }
    // remove the oversampler latency so the signals line up with the others
    const int lat = amp.getLatencySamples();
    y.erase (y.begin(), y.begin() + lat);
    y.resize (x.size(), 0.0);
    return y;
}

// from fs * os down to fs with a 2047-tap windowed sinc (20 kHz)
Signal decimate (const Signal& y, int os)
{
    const double rate = fs * os;
    constexpr int taps = 2047;
    std::vector<double> h (taps);
    double sum = 0;
    for (int i = 0; i < taps; ++i)
    {
        const double m = i - (taps - 1) / 2.0, fc = 20000.0 / rate;
        h[(size_t) i] = (m == 0 ? 2 * fc : std::sin (2 * kPi * fc * m) / (kPi * m))
                      * (0.42 - 0.5 * std::cos (2 * kPi * i / (taps - 1)) + 0.08 * std::cos (4 * kPi * i / (taps - 1)));
        sum += h[(size_t) i];
    }
    Signal out (y.size() / (size_t) os);
    for (size_t o = 0; o < out.size(); ++o)
    {
        double acc = 0;
        for (int i = 0; i < taps; ++i)
        {
            const long k = (long) (o * (size_t) os) - i + (taps - 1) / 2;
            if (k >= 0 && k < (long) y.size()) acc += h[(size_t) i] * y[(size_t) k];
        }
        out[o] = acc / sum;
    }
    return out;
}

// ideal: the same DSP classes at 16x, decimated with a 2047-tap windowed sinc (20 kHz)
Signal idealPath (const Gen& g, double secs, int os = 16)
{
    const double rate = fs * os;
    const auto s = settings();
    vamp::Preamp5150 pre; pre.prepare (rate); pre.setMode (s.channel);
    pre.setControls (s.gain, s.bass, s.mid, s.treble, s.master);
    vamp::PowerAmp5150 pa; pa.setControls (s.presence, s.resonance); pa.prepare (rate);
    const auto x = sample (g, rate, secs);
    Signal y (x.size());
    for (size_t i = 0; i < x.size(); ++i) y[i] = vamp::processAmp5150 (pre, pa, x[i], 1.0) * vamp::kSpeakerToDigital;
    return decimate (y, os);
}

double voltsPerFS = 4.12;
Signal namPath (const namcmp::NamWaveNet& model, const Gen& g, double secs)
{
    namcmp::NamWaveNet m = model;
    for (int i = 0; i < 4096; ++i) m.process (0.0f);
    const auto x = sample (g, fs, secs);
    Signal y (x.size());
    for (size_t i = 0; i < x.size(); ++i) y[i] = m.process ((float) (x[i] / voltsPerFS));
    return y;
}

double db (double v) { return 20.0 * std::log10 (std::max (v, 1e-15)); }

// power spectrum of the last 2^16 samples, Blackman-Harris window
std::vector<double> spectrum (const Signal& y)
{
    constexpr int n = 1 << 16;
    vamp::FFT fft; fft.prepare (n);
    std::vector<std::complex<double>> d ((size_t) n);
    const size_t off = y.size() - n;
    for (int i = 0; i < n; ++i)
    {
        const double w = 0.35875 - 0.48829 * std::cos (2 * kPi * i / n) + 0.14128 * std::cos (4 * kPi * i / n)
                       - 0.01168 * std::cos (6 * kPi * i / n);
        d[(size_t) i] = y[off + (size_t) i] * w;
    }
    fft.transform (d.data(), false);
    std::vector<double> p ((size_t) n / 2);
    for (int k = 0; k < n / 2; ++k) p[(size_t) k] = std::norm (d[(size_t) k]);
    return p;
}

// inharmonic / total energy in 20 Hz .. 20 kHz (harmonics = bins within 6 of k*f0)
double inharmonicDb (const Signal& y, double f0, double tolHz = 0.0) // tolHz: harmonic zone half-width (0: 6 bins)
{
    const auto p = spectrum (y);
    const double binHz = fs / (2.0 * (double) p.size());
    double tot = 0, inh = 0;
    for (size_t k = 0; k < p.size(); ++k)
    {
        const double f = k * binHz;
        if (f < 20.0 || f > 20000.0) continue;
        tot += p[k];
        const double h = std::round (f / f0);
        if (h < 1.0 || std::abs (f - h * f0) > (tolHz > 0.0 ? tolHz : 6.0 * binHz)) inh += p[k];
    }
    return 10.0 * std::log10 (std::max (inh, 1e-30) / tot);
}

std::vector<double> bands (const Signal& x, double from = 0.2)
{
    std::vector<double> out;
    for (double fc = 63.0; fc < 17000.0; fc *= std::pow (2.0, 1.0 / 3.0))
    {
        const double w0 = 2 * kPi * fc / fs, al = std::sin (w0) / (2 * 4.318), c = std::cos (w0), a0 = 1 + al;
        const double b0 = al / a0, b2 = -al / a0, a1 = -2 * c / a0, a2 = (1 - al) / a0;
        double z1 = 0, z2 = 0, e = 0;
        for (size_t i = 0; i < x.size(); ++i)
        {
            const double y = b0 * x[i] + z1;
            z1 = -a1 * y + z2; z2 = b2 * x[i] - a2 * y;
            if (i >= (size_t) (from * fs)) e += y * y;
        }
        out.push_back (e);
    }
    return out;
}

double pluck (double t, double f0, double decay, double amp)
{
    double v = 0;
    for (int h = 1; h <= 8; ++h) v += amp / h * std::exp (-t * decay * (1.0 + 0.3 * h)) * std::sin (2 * kPi * f0 * h * t + h);
    return v;
}

// pick attack: a 1.5 ms burst of high-passed noise at each pluck (deterministic)
double pickNoise (double t)
{
    if (t < 0.0 || t > 0.0015) return 0.0;
    const unsigned u = (unsigned) (t * 1e7) * 2654435761u;
    return 0.04 * ((u >> 8) / 8388608.0 - 1.0) * (1.0 - t / 0.0015);
}
// Zipper noise while a knob moves: a 220 Hz sine through the plugin path, one knob automated linearly from a to b
// over 0.4 s in the middle of the analysis window (the host delivers it once per block). Without zipper noise the
// output is the 220 Hz harmonic series with a slowly changing level; steps add energy between the harmonics.
// knob: 0 gain, 4 master, 5 presence, 6 resonance (AmpSim::Settings order). Returns inharmonic energy (dB re total).
double zipperDb (int knob, double a, double b, int blockSize, double smoothMs)
{
    vamp::AmpSim amp;
    amp.prepare (fs, blockSize);
    amp.setKnobSmoothing (smoothMs);
    auto s = settings();
    auto set = [&] (double v) { float* k[] = { &s.gain, nullptr, nullptr, nullptr, &s.master, &s.presence, &s.resonance }; *k[knob] = (float) v; };
    set (a);
    std::vector<float> buf ((size_t) blockSize);
    auto block = [&] (long long& n, Signal* out)
    {
        for (int i = 0; i < blockSize; ++i) buf[(size_t) i] = (float) (0.1 * std::sin (2 * kPi * 220.0 * (double) (n + i) / fs));
        amp.process (buf.data(), blockSize, s);
        if (out) for (float v : buf) out->push_back (v);
        n += blockSize;
    };
    long long n = 0;
    while (n < (long long) fs) block (n, nullptr); // settle at a
    Signal y;
    const long long start = n, len = 1 << 16, rampFrom = start + len / 2 - (long long) (0.2 * fs), rampTo = rampFrom + (long long) (0.4 * fs);
    while (n < start + len)
    {
        set (n < rampFrom ? a : n > rampTo ? b : a + (b - a) * (double) (n - rampFrom) / (double) (rampTo - rampFrom));
        block (n, &y);
    }
    return inharmonicDb (y, 220.0, 20.0); // +-20 Hz: the level change itself widens each harmonic by a few Hz
}

void zipperTests()
{
    std::printf ("\n== zipper noise: knob automated over 0.4 s, 220 Hz 0.1 V in, inharmonic energy re total (dB, lower is cleaner) ==\n");
    std::printf ("%-28s %7s | %12s %12s\n", "knob", "block", "block ramp", "+ 20 ms");
    struct K { const char* name; int knob; double a, b; };
    for (const auto& k : { K { "Master 0.2 -> 0.6", 4, 0.2, 0.6 }, K { "Gain 0.3 -> 0.7", 0, 0.3, 0.7 },
                           K { "Presence 0 -> 1", 5, 0.0, 1.0 }, K { "Resonance 0 -> 1", 6, 0.0, 1.0 } })
        for (int bs : { 512, 64 })
            std::printf ("%-28s %7d | %+12.1f %+12.1f\n", k.name, bs, zipperDb (k.knob, k.a, k.b, bs, 0.0), zipperDb (k.knob, k.a, k.b, bs, 20.0));
    std::printf ("(reference: knob held still)  512 | %+12.1f\n", zipperDb (4, 0.3, 0.3, 512, 20.0));
}

// Channel switch: 220 Hz 0.1 V sine at 8x (the plugin's internal rate), analysed after decimation to 48 kHz.
// Pre-roll: channel `to` for 0.5 s, then `from` for 1 s (so `to` comes back after having been left mid-signal), then
// a 2^16-sample window with the switch to `to` in the middle.
// Reference: what a switch can do at best: two more preamps, each always running its channel (warm, steady),
// crossfaded over 10 ms and fed to one power amp. The power amp's own response to a level change (it re-biases when
// the drive jumps, as the real amp does) is in both. Returns { inharmonic dB (clicks), energy below 40 Hz dB re
// total (thumps) } for the switch and for the reference.
struct SwitchResult { double click, thump, refClick, refThump; };

SwitchResult switchTest (int from, int to, double fadeMs)
{
    constexpr int os = 8;
    const double rate = fs * os;
    const long long pre1 = (long long) (0.5 * rate), pre2 = pre1 + (long long) rate, len = (1 << 16) * (long long) os;
    const long long sw = pre2 + len / 2;
    const auto s = settings();
    auto makePre = [&] (int channel)
    {
        auto p = std::make_unique<vamp::Preamp5150>();
        p->setChannelTrims (vamp::kChannelPost[0], vamp::kChannelPost[1], vamp::kChannelPost[2]);
        p->switchFadeMs = fadeMs;
        p->prepare (rate); p->setMode (channel);
        p->setControls (s.gain, s.bass, s.mid, s.treble, s.master);
        return p;
    };
    auto makePa = [&] { auto p = std::make_unique<vamp::PowerAmp5150>(); p->setControls (s.presence, s.resonance); p->prepare (rate); return p; };
    auto in = [&] (long long n) { return 0.1 * std::sin (2 * kPi * 220.0 * (double) n / rate); };

    Signal y, ref;
    {
        auto pre = makePre (to); auto pa = makePa();
        for (long long n = 0; n < pre2 + len; ++n)
        {
            if (n == pre1) pre->setMode (from);
            if (n == sw) pre->setMode (to);
            const double v = vamp::processAmp5150 (*pre, *pa, in (n), 1.0);
            if (n >= pre2) y.push_back (v * vamp::kSpeakerToDigital);
        }
    }
    {
        auto a = makePre (from), b = makePre (to); auto pa = makePa();
        const double fade = fadeMs * 1e-3 * rate;
        for (long long n = 0; n < pre2 + len; ++n)
        {
            const double t = (double) (n - sw) / std::max (fade, 1.0), w = t <= 0.0 ? 0.0 : t >= 1.0 ? 1.0 : 0.5 - 0.5 * std::cos (kPi * t);
            for (auto* p : { a.get(), b.get() }) p->setRails (pa->getRailV1(), pa->getRailV2());
            const double x = (1.0 - w) * a->process (in (n)) + w * b->process (in (n));
            double i1a, i2a, i1b, i2b;
            a->railCurrents (i1a, i2a); b->railCurrents (i1b, i2b);
            pa->setPreampLoad ((1.0 - w) * i1a + w * i1b, (1.0 - w) * i2a + w * i2b);
            const double v = pa->process (x);
            if (n >= pre2) ref.push_back (v * vamp::kSpeakerToDigital);
        }
    }
    y = decimate (y, os);
    ref = decimate (ref, os);
    auto lowDb = [] (const Signal& x)
    {
        const auto p = spectrum (x);
        const double binHz = fs / (2.0 * (double) p.size());
        double tot = 0, low = 0;
        for (size_t k = 1; k < p.size(); ++k) { tot += p[k]; if (k * binHz < 40.0) low += p[k]; }
        return 10.0 * std::log10 (std::max (low, 1e-30) / tot);
    };
    return { inharmonicDb (y, 220.0, 20.0), lowDb (y), inharmonicDb (ref, 220.0, 20.0), lowDb (ref) };
}

// A third channel mid-fade: `a` (pre-roll 1 s), the switch to `b`, then to `c` gapMs later (before the first fade
// ends). Reference: three always-running preamps mixed with the same weights (the weights of the moment fade to the
// new channel: Preamp5150::setMode), fed to one power amp. Same analysis as switchTest.
SwitchResult switchTest3 (int a, int b, int c, double fadeMs, double gapMs)
{
    constexpr int os = 8;
    const double rate = fs * os;
    const long long pre = (long long) rate, len = (1 << 16) * (long long) os;
    const long long sw1 = pre + len / 2, sw2 = sw1 + (long long) (gapMs * 1e-3 * rate);
    const int fadeLen = std::max (1, (int) (fadeMs * 1e-3 * rate));
    const auto s = settings();
    auto makePre = [&] (int channel)
    {
        auto p = std::make_unique<vamp::Preamp5150>();
        p->setChannelTrims (vamp::kChannelPost[0], vamp::kChannelPost[1], vamp::kChannelPost[2]);
        p->switchFadeMs = fadeMs;
        p->prepare (rate); p->setMode (channel);
        p->setControls (s.gain, s.bass, s.mid, s.treble, s.master);
        return p;
    };
    auto makePa = [&] { auto p = std::make_unique<vamp::PowerAmp5150>(); p->setControls (s.presence, s.resonance); p->prepare (rate); return p; };
    auto in = [&] (long long n) { return 0.1 * std::sin (2 * kPi * 220.0 * (double) n / rate); };

    Signal y, ref;
    {
        auto p = makePre (a); auto pa = makePa();
        for (long long n = 0; n < pre + len; ++n)
        {
            if (n == sw1) p->setMode (b);
            if (n == sw2) p->setMode (c);
            const double v = vamp::processAmp5150 (*p, *pa, in (n), 1.0);
            if (n >= pre) y.push_back (v * vamp::kSpeakerToDigital);
        }
    }
    {
        std::unique_ptr<vamp::Preamp5150> ps[3] = { makePre (a), makePre (b), makePre (c) };
        auto pa = makePa();
        auto ramp = [&] (long long n, long long sw) { const double t = (double) (n - sw + 1) / fadeLen; return t <= 0.0 ? 0.0 : t >= 1.0 ? 1.0 : 0.5 - 0.5 * std::cos (kPi * t); };
        double w[3] = { 1.0, 0.0, 0.0 }, w0[3] = { 1.0, 0.0, 0.0 };
        for (long long n = 0; n < pre + len; ++n)
        {
            if (n == sw2) std::copy (std::begin (w), std::end (w), std::begin (w0));
            if (n >= sw2) { const double r = ramp (n, sw2); for (int k = 0; k < 3; ++k) w[k] = w0[k] * (1.0 - r) + (k == 2 ? r : 0.0); }
            else if (n >= sw1) { const double r = ramp (n, sw1); w[0] = 1.0 - r; w[1] = r; w[2] = 0.0; }
            double x = 0.0, i1 = 0.0, i2 = 0.0;
            for (int k = 0; k < 3; ++k)
            {
                ps[k]->setRails (pa->getRailV1(), pa->getRailV2());
                x += w[k] * ps[k]->process (in (n));
                double c1, c2; ps[k]->railCurrents (c1, c2);
                i1 += w[k] * c1; i2 += w[k] * c2;
            }
            pa->setPreampLoad (i1, i2);
            const double v = pa->process (x);
            if (n >= pre) ref.push_back (v * vamp::kSpeakerToDigital);
        }
    }
    y = decimate (y, os);
    ref = decimate (ref, os);
    auto lowDb = [] (const Signal& x)
    {
        const auto p = spectrum (x);
        const double binHz = fs / (2.0 * (double) p.size());
        double tot = 0, low = 0;
        for (size_t k = 1; k < p.size(); ++k) { tot += p[k]; if (k * binHz < 40.0) low += p[k]; }
        return 10.0 * std::log10 (std::max (low, 1e-30) / tot);
    };
    return { inharmonicDb (y, 220.0, 20.0), lowDb (y), inharmonicDb (ref, 220.0, 20.0), lowDb (ref) };
}

// CRUNCH flipped mid-note on BLUE (relay K4): as switchTest, the reference two always-running preamps (BLUE with
// crunch from / to) crossfaded over the same time
SwitchResult crunchTest (bool from, double fadeMs)
{
    constexpr int os = 8;
    const double rate = fs * os;
    const long long pre1 = (long long) (0.5 * rate), pre2 = pre1 + (long long) rate, len = (1 << 16) * (long long) os;
    const long long sw = pre2 + len / 2;
    const auto s = settings();
    auto makePre = [&] (bool crunch)
    {
        auto p = std::make_unique<vamp::Preamp5150> (false, crunch);
        p->setChannelTrims (vamp::kChannelPost[0], vamp::kChannelPost[1], vamp::kChannelPost[2]);
        p->switchFadeMs = fadeMs;
        p->prepare (rate); p->setMode (vamp::Preamp5150::Blue);
        p->setControls (s.gain, s.bass, s.mid, s.treble, s.master);
        return p;
    };
    auto makePa = [&] { auto p = std::make_unique<vamp::PowerAmp5150>(); p->setControls (s.presence, s.resonance); p->prepare (rate); return p; };
    auto in = [&] (long long n) { return 0.1 * std::sin (2 * kPi * 220.0 * (double) n / rate); };

    Signal y, ref;
    {
        auto pre = makePre (! from); auto pa = makePa();
        for (long long n = 0; n < pre2 + len; ++n)
        {
            if (n == pre1) pre->setCrunch (from);
            if (n == sw) pre->setCrunch (! from);
            const double v = vamp::processAmp5150 (*pre, *pa, in (n), 1.0);
            if (n >= pre2) y.push_back (v * vamp::kSpeakerToDigital);
        }
    }
    {
        auto a = makePre (from), b = makePre (! from); auto pa = makePa();
        const double fade = fadeMs * 1e-3 * rate;
        for (long long n = 0; n < pre2 + len; ++n)
        {
            const double t = (double) (n - sw) / std::max (fade, 1.0), w = t <= 0.0 ? 0.0 : t >= 1.0 ? 1.0 : 0.5 - 0.5 * std::cos (kPi * t);
            for (auto* p : { a.get(), b.get() }) p->setRails (pa->getRailV1(), pa->getRailV2());
            const double x = (1.0 - w) * a->process (in (n)) + w * b->process (in (n));
            double i1a, i2a, i1b, i2b;
            a->railCurrents (i1a, i2a); b->railCurrents (i1b, i2b);
            pa->setPreampLoad ((1.0 - w) * i1a + w * i1b, (1.0 - w) * i2a + w * i2b);
            const double v = pa->process (x);
            if (n >= pre2) ref.push_back (v * vamp::kSpeakerToDigital);
        }
    }
    y = decimate (y, os);
    ref = decimate (ref, os);
    auto lowDb = [] (const Signal& x)
    {
        const auto p = spectrum (x);
        const double binHz = fs / (2.0 * (double) p.size());
        double tot = 0, low = 0;
        for (size_t k = 1; k < p.size(); ++k) { tot += p[k]; if (k * binHz < 40.0) low += p[k]; }
        return 10.0 * std::log10 (std::max (low, 1e-30) / tot);
    };
    return { inharmonicDb (y, 220.0, 20.0), lowDb (y), inharmonicDb (ref, 220.0, 20.0), lowDb (ref) };
}

// Sustain: a 110 Hz note (8 harmonics) picked at peakV and decaying at decayDbPerS, through the plugin amp path
// (gate off and with the default 3-band gate keyed by the input) and the NAM capture. Output level (RMS over 100 ms)
// every 0.5 s, dB re the level 0.2 s after the pick.
void sustainTest (const namcmp::NamWaveNet* nam, double peakV, double decayDbPerS)
{
    const double secs = 8.0;
    const Gen note = [peakV, decayDbPerS] (double t)
    {
        const double env = std::pow (10.0, -decayDbPerS * t / 20.0) * (1.0 - std::exp (-t / 0.002));
        double v = 0;
        for (int h = 1; h <= 8; ++h) v += std::sin (2 * kPi * 110.0 * h * t + h) / (h * h);
        return peakV * env * v / 1.5;
    };
    const auto x = sample (note, fs, secs);
    const auto amp = pluginPath (note, secs);
    Signal gated = amp;
    {
        vamp::MultibandGate gate;
        gate.prepare (fs);
        std::vector<float> a (gated.begin(), gated.end()), k (x.begin(), x.end());
        gate.process (a.data(), k.data(), (int) a.size(), vamp::MultibandGate::Settings {});
        std::copy (a.begin(), a.end(), gated.begin());
    }
    Signal namOut;
    if (nam) namOut = namPath (*nam, note, secs);

    auto level = [] (const Signal& y, double t)
    {
        double e = 0; int n = 0;
        for (size_t i = (size_t) (t * fs); i < (size_t) ((t + 0.1) * fs) && i < y.size(); ++i) { e += y[i] * y[i]; ++n; }
        return 10.0 * std::log10 (std::max (e / std::max (n, 1), 1e-30));
    };
    std::printf ("\n== sustain: 110 Hz note, %.2f V peak, input decaying %.0f dB/s (output dB re 0.2 s after the pick) ==\n", peakV, decayDbPerS);
    std::printf ("%6s %8s | %10s %10s %10s\n", "t (s)", "input", "amp", "amp+gate", nam ? "NAM 6505" : "");
    const double x0 = level (x, 0.2), a0 = level (amp, 0.2), g0 = level (gated, 0.2), n0 = nam ? level (namOut, 0.2) : 0.0;
    for (double t = 0.2; t < secs - 0.1; t += 0.5)
    {
        std::printf ("%6.1f %+8.1f | %+10.1f %+10.1f", t, level (x, t) - x0, level (amp, t) - a0, level (gated, t) - g0);
        if (nam) std::printf (" %+10.1f", level (namOut, t) - n0);
        std::printf ("\n");
    }
}

void switchTests()
{
    std::printf ("\n== channel switch: 220 Hz 0.1 V; dB re total (lower is cleaner) ==\n");
    std::printf ("(ideal: two always-running preamps crossfaded over the same time into one power amp)\n");
    const char* name[] = { "CLEAN", "BLUE", "RED" };
    for (double fadeMs : { 0.0, 20.0 }) // 0: a hard switch (one sample), for comparison
    {
        std::printf ("%-11s %4.0f ms | %10s %10s | %10s %10s\n", "switch, fade", fadeMs, "clicks", "thump", "ideal clk", "ideal thp");
        for (const auto& p : { std::pair { 2, 1 }, std::pair { 1, 2 }, std::pair { 2, 0 }, std::pair { 0, 2 }, std::pair { 1, 0 }, std::pair { 0, 1 } })
        {
            const auto r = switchTest (p.first, p.second, fadeMs);
            std::printf ("%-6s -> %-11s | %+10.1f %+10.1f | %+10.1f %+10.1f\n", name[p.first], name[p.second], r.click, r.thump, r.refClick, r.refThump);
        }
    }
    std::printf ("third channel 8 ms into a 20 ms fade | %10s %10s | %10s %10s\n", "clicks", "thump", "ideal clk", "ideal thp");
    for (const auto& t : { std::array { 2, 1, 0 }, std::array { 0, 1, 2 }, std::array { 1, 2, 0 }, std::array { 2, 0, 1 } })
    {
        const auto r = switchTest3 (t[0], t[1], t[2], 20.0, 8.0);
        std::printf ("%-6s -> %-6s -> %-14s | %+10.1f %+10.1f | %+10.1f %+10.1f\n", name[t[0]], name[t[1]], name[t[2]], r.click, r.thump, r.refClick, r.refThump);
    }
    std::printf ("BLUE, CRUNCH flipped, 20 ms fade     | %10s %10s | %10s %10s\n", "clicks", "thump", "ideal clk", "ideal thp");
    for (bool from : { true, false })
    {
        const auto r = crunchTest (from, 20.0);
        std::printf ("crunch %-3s -> %-22s | %+10.1f %+10.1f | %+10.1f %+10.1f\n", from ? "on" : "off", from ? "off" : "on", r.click, r.thump, r.refClick, r.refThump);
    }
}
} // namespace

int main (int argc, char** argv)
{
    if (argc > 1 && std::string (argv[1]) == "knobs") { zipperTests(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "switch") { switchTests(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "sustain") // [model.nam]: the capture of settings() (Pr-5 B-6 M-4 T-6 Po-3 R-9 P-8)
    {
        namcmp::NamWaveNet nam;
        const bool have = argc > 2 && nam.load (argv[2]);
        for (double v : { 0.3, 0.1 }) sustainTest (have ? &nam : nullptr, v, 12.0);
        return 0;
    }
    if (argc > 1 && std::string (argv[1]) == "prepare")
    {
        std::printf ("\n== AmpSim::prepare() time (the host waits for it on load and on sample-rate / buffer changes) ==\n");
        for (double rate : { 44100.0, 48000.0, 96000.0, 192000.0 })
        {
            vamp::AmpSim amp;
            const auto t0 = std::chrono::high_resolution_clock::now();
            amp.prepare (rate, 512);
            const double ms = std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - t0).count();
            std::printf ("  %6.0f Hz: %7.0f ms\n", rate, ms);
        }
        return 0;
    }
    namcmp::NamWaveNet nam;
    const bool haveNam = argc > 1 && nam.load (argv[1]);
    if (argc > 2) voltsPerFS = 0.7746 * std::pow (10.0, std::atof (argv[2]) / 20.0) * std::sqrt (2.0);
    std::printf ("plugin path = AmpSim at 48 kHz (JUCE IIR oversampling, %d samples latency); ideal = N x + linear-phase decimation%s\n",
                 [] { vamp::AmpSim a; a.prepare (fs, 64); return a.getLatencySamples(); }(),
                 haveNam ? "; NAM = real 6505 capture" : "");

    // 1. aliasing
    std::printf ("\n== 1. aliasing: inharmonic energy re total, 20 Hz..20 kHz (lower is cleaner) ==\n");
    std::printf ("%-18s %10s %10s %10s %10s %10s\n", "sine", "plugin", "4x+FIR", "8x+FIR", "16x+FIR", "NAM");
    for (double f0 : { 997.0, 2003.0, 3001.0, 4999.0 })
        for (double a : { 0.05, 0.3 })
        {
            const Gen g = [f0, a] (double t) { return a * std::sin (2 * kPi * f0 * t); };
            const double p = inharmonicDb (pluginPath (g, 2.0), f0), i4 = inharmonicDb (idealPath (g, 2.0, 4), f0), i8 = inharmonicDb (idealPath (g, 2.0, 8), f0), id = inharmonicDb (idealPath (g, 2.0), f0);
            std::printf ("%5.0f Hz %5.2f V  %+9.1f  %+9.1f  %+9.1f  %+9.1f", f0, a, p, i4, i8, id);
            if (haveNam) std::printf ("  %+9.1f", inharmonicDb (namPath (nam, g, 2.0), f0));
            std::printf ("\n");
        }

    // 2. plugin path vs ideal on guitar-like input
    const Gen chug = [] (double t)
    {
        const double ph = std::fmod (t, 0.5), tc = std::fmod (ph, 0.125);
        double v = 0;
        for (double f : { 36.7, 55.0, 73.4 }) v += pluck (tc, f, 25.0, 0.15);
        return v + pickNoise (tc);
    };
    const auto yp = pluginPath (chug, 2.0), yi = idealPath (chug, 2.0);
    const auto bp = bands (yp), bi = bands (yi);
    std::printf ("\n== 2. tone of the plugin path vs ideal (chugs with pick noise), 1/3 octave, dB ==\n");
    double worst = 0; int bi_ = 0;
    for (double fc = 63.0; fc < 17000.0; fc *= std::pow (2.0, 1.0 / 3.0), ++bi_)
    {
        const double d = 10 * std::log10 (bp[(size_t) bi_] / bi[(size_t) bi_]);
        worst = std::max (worst, std::abs (d));
        if (bi_ % 3 == 0) std::printf ("  %5.0f Hz %+5.2f", fc, d);
    }
    std::printf ("\n  max |difference| over all bands: %.2f dB\n", worst);

    // CPU of the whole plugin amp path (oversampling filters included), 10 s of chugs, 64-sample blocks; knobs still,
    // then Gain and Presence moving all the time (0.5 Hz LFO): each smoothing step rebuilds the lead preamp's and
    // the power amp's circuit matrices
    for (const bool moving : { false, true })
    {
        vamp::AmpSim amp;
        amp.prepare (fs, 64);
        auto s = settings();
        const auto x = sample (chug, fs, 10.0);
        std::vector<float> buf (64);
        double total = 0, worst = 0;
        for (size_t start = 0; start + 64 <= x.size(); start += 64)
        {
            if (moving)
            {
                const double lfo = 0.5 + 0.5 * std::sin (2 * kPi * 0.5 * (double) start / fs);
                s.gain = (float) (0.3 + 0.4 * lfo);
                s.presence = (float) lfo;
            }
            for (int i = 0; i < 64; ++i) buf[(size_t) i] = (float) x[start + (size_t) i];
            const auto t0 = std::chrono::high_resolution_clock::now();
            amp.process (buf.data(), 64, s);
            const double us = std::chrono::duration<double, std::micro> (std::chrono::high_resolution_clock::now() - t0).count();
            total += us; worst = std::max (worst, us);
        }
        std::printf ("\n== CPU of the plugin amp path (RED, gain 5, master 3%s): avg %.1f%% of one core, worst 64-sample block %.0f%% ==\n",
                     moving ? ", Gain + Presence moving" : "", 100.0 * total / 10e6, 100.0 * worst / (64.0 / fs * 1e6));
    }

    if (! haveNam) return 0;

    // 3. pick attack / fizz vs NAM: band levels re the 1 kHz band, plugin vs NAM
    const auto yn = namPath (nam, chug, 2.0);
    const auto bn = bands (yn);
    std::printf ("\n== 3. pick attack & fizz: band level re 1 kHz band, plugin vs NAM (chugs with pick noise) ==\n");
    int k1 = 0; { int k = 0; for (double fc = 63.0; fc < 17000.0; fc *= std::pow (2.0, 1.0 / 3.0), ++k) if (std::abs (fc / 1000.0 - 1) < 0.06) k1 = k; }
    bi_ = 0;
    for (double fc = 63.0; fc < 17000.0; fc *= std::pow (2.0, 1.0 / 3.0), ++bi_)
    {
        if (fc < 1900.0) continue;
        const double op = 10 * std::log10 (bp[(size_t) bi_] / bp[(size_t) k1]), on = 10 * std::log10 (bn[(size_t) bi_] / bn[(size_t) k1]);
        std::printf ("  %5.0f Hz: plugin %+6.1f  NAM %+6.1f  diff %+5.1f\n", fc, op, on, op - on);
    }

    // 4. dynamics: tone burst quiet (0.03 V) / loud (0.3 V) / quiet, 10 ms RMS envelope
    const Gen burst = [] (double t)
    {
        const double a = (t >= 0.5 && t < 1.0) ? 0.3 : 0.03;
        return a * std::sin (2 * kPi * 110.0 * t);
    };
    const auto ep = pluginPath (burst, 1.6), en = namPath (nam, burst, 1.6);
    auto env = [] (const Signal& y, double t) { double s = 0; const size_t n = (size_t) (4.0 * fs / 110.0), a = (size_t) (t * fs) - n / 2; for (size_t i = a; i < a + n; ++i) s += y[i] * y[i]; return std::sqrt (s / n); };
    std::printf ("\n== 4. dynamics: 110 Hz burst 0.03 V -> 0.3 V (at 0.5 s) -> 0.03 V (at 1.0 s), 4-period RMS re end of loud part (dB) ==\n");
    std::printf ("%8s %8s %8s\n", "time", "plugin", "NAM");
    const double rp = env (ep, 0.98), rn = env (en, 0.98);
    for (double t : { 0.40, 0.49, 0.505, 0.52, 0.55, 0.60, 0.70, 0.90, 0.98, 1.005, 1.02, 1.05, 1.10, 1.20, 1.40, 1.58 })
        std::printf ("%7.3fs %+8.2f %+8.2f\n", t, db (env (ep, t) / rp), db (env (en, t) / rn));
    return 0;
}
