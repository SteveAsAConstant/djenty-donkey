// CPU of the whole plugin (VibeAmpProcessor::processBlock) at 48 kHz, per buffer size and per stage:
//
//   plugin_cpu [ir.wav ...]     (pin it to a P-core: start /high /affinity 1000)
//   plugin_cpu levels [ir.wav ...]   the output level with each IR vs the built-in cab (they are loudness-matched)
//   plugin_cpu upgrade               a state from before the 1176's DI is converted (INPUT / OUTPUT)
//
// 1. the default patch (gate + cab on) and everything on (1176, drive, gate, LIVE transpose, cab, EQ, phaser,
//    delay, reverb), with each cab IR given (and the built-in fallback cab)
// 2. per stage: everything on, minus that stage (its own parameter switched off): the difference is its cost
// 3. inside the amp: AmpSim (JUCE polyphase-IIR oversampling + the circuits) vs the circuits alone at 8x, and
//    Comp1176 (2x oversampling + the 1176) vs the 1176 alone
// Each case runs kRuns times from a freshly prepared processor; per block the fastest run is kept (drops OS
// interruptions). avg = % of real time (one core); worst = the slowest block as % of its own duration.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <juce_audio_processors/juce_audio_processors.h>
#include "PluginProcessor.h"
#include "dsp/AmpSim.h"
#include "dsp/Comp1176.h"

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter();

namespace
{
constexpr double kRate = 48000.0, kSeconds = 6.0;
constexpr int kRuns = 3;
using clk = std::chrono::steady_clock;

// guitar-like input at ~0.5 V peak: drop-A palm-muted chugs with pick noise, a ringing chord every 2 s
float guitar (double t)
{
    const double ph = std::fmod (t, 2.0), tc = std::fmod (ph, 0.125);
    double v = 0;
    const bool ring = ph >= 1.5;
    const double tt = ring ? ph - 1.5 : tc, decay = ring ? 2.5 : 25.0;
    for (double f : { 55.0, 82.41, 110.0 })
        for (int h = 1; h <= 12; ++h) v += 0.06 / h * std::exp (-tt * decay * (1.0 + 0.3 * h)) * std::sin (2 * juce::MathConstants<double>::pi * f * h * t + h);
    if (tt < 0.0015) v += 0.1 * std::sin (t * 123456.789) * (1.0 - tt / 0.0015);
    return (float) v;
}

void set (juce::AudioProcessor& p, const char* id, float value)
{
    for (auto* prm : p.getParameters())
        if (auto* r = dynamic_cast<juce::RangedAudioParameter*> (prm))
            if (r->getParameterID() == id) { r->setValueNotifyingHost (r->convertTo0to1 (value)); return; }
    std::printf ("(no parameter %s)\n", id);
}

struct Result { double avg, worst; };

// per-block times (fastest of kRuns) -> avg % of real time and the worst block as % of its duration
Result summarise (const std::vector<double>& best, int block)
{
    const double blockUs = 1e6 * block / kRate;
    double sum = 0, worst = 0;
    const size_t skip = (size_t) (0.5 * kRate / block); // the first 0.5 s (warm-up) left out
    for (size_t b = skip; b < best.size(); ++b) { sum += best[b]; worst = std::max (worst, best[b]); }
    return { 100.0 * sum / (blockUs * (double) (best.size() - skip)), 100.0 * worst / blockUs };
}

using Patch = std::function<void (juce::AudioProcessor&)>;

Result runPlugin (const Patch& patch, const juce::String& ir, int block)
{
    std::vector<double> best;
    for (int r = 0; r < kRuns; ++r)
    {
        std::unique_ptr<juce::AudioProcessor> p (createPluginFilter());
        juce::AudioProcessor::BusesLayout layout;
        layout.inputBuses.add (juce::AudioChannelSet::mono());
        layout.outputBuses.add (juce::AudioChannelSet::stereo());
        p->setBusesLayout (layout);
        for (auto* prm : p->getParameters()) prm->setValueNotifyingHost (prm->getDefaultValue());
        patch (*p);
        p->setRateAndBufferSizeDetails (kRate, block);
        p->prepareToPlay (kRate, block);
        if (ir.isNotEmpty()) dynamic_cast<VibeAmpProcessor&> (*p).loadIR (juce::File (ir));

        juce::AudioBuffer<float> buf (2, block);
        juce::MidiBuffer midi;
        // warm-up: the IR is resampled / partitioned on JUCE's background thread; give it time to swap in
        for (int k = 0; k < 6; ++k)
        {
            for (int b = 0; b < (int) (0.1 * kRate) / block; ++b) { buf.clear(); p->processBlock (buf, midi); }
            std::this_thread::sleep_for (std::chrono::milliseconds (150));
        }
        const int numBlocks = (int) (kSeconds * kRate) / block;
        if (best.empty()) best.assign ((size_t) numBlocks, 1e30);
        for (int b = 0; b < numBlocks; ++b)
        {
            for (int i = 0; i < block; ++i) { const float v = guitar ((double) (b * block + i) / kRate); buf.setSample (0, i, v); buf.setSample (1, i, 0.0f); }
            const auto t0 = clk::now();
            p->processBlock (buf, midi);
            best[(size_t) b] = std::min (best[(size_t) b], std::chrono::duration<double, std::micro> (clk::now() - t0).count());
        }
    }
    return summarise (best, block);
}

// a module on its own: process (data, n) on fresh instances, fastest of kRuns per block
template <typename Make>
Result runModule (Make make, int block)
{
    std::vector<double> best;
    for (int r = 0; r < kRuns; ++r)
    {
        auto proc = make();
        const int numBlocks = (int) (kSeconds * kRate) / block;
        if (best.empty()) best.assign ((size_t) numBlocks, 1e30);
        std::vector<float> buf ((size_t) block);
        for (int b = 0; b < numBlocks; ++b)
        {
            for (int i = 0; i < block; ++i) buf[(size_t) i] = guitar ((double) (b * block + i) / kRate);
            const auto t0 = clk::now();
            proc (buf.data(), block);
            best[(size_t) b] = std::min (best[(size_t) b], std::chrono::duration<double, std::micro> (clk::now() - t0).count());
        }
    }
    return summarise (best, block);
}

void everythingOn (juce::AudioProcessor& p)
{
    set (p, ids::compOn, 1); set (p, ids::boostOn, 1); set (p, ids::gateOn, 1); set (p, ids::cabOn, 1);
    set (p, ids::eqOn, 1); set (p, ids::phaserOn, 1); set (p, ids::delayOn, 1); set (p, ids::reverbOn, 1);
    set (p, ids::transpose, -2); set (p, ids::transposeMode, 0);
}
} // namespace

// plugin_cpu levels [ir.wav ...]: the default patch's output level with the built-in cab and with each IR
// (plain RMS and K-weighted RMS of 8 s of chugs and of a lead line, dB re the built-in cab): loaded IRs are matched
// to the built-in cab
float leadLine (double t) // E-minor notes 330..990 Hz, one every 0.3 s, ~0.4 V peak
{
    const double notes[] = { 329.6, 392.0, 440.0, 587.3, 659.3, 784.0, 880.0, 987.8 };
    const double f = notes[(int) (t / 0.3) % 8], tn = std::fmod (t, 0.3);
    double v = 0;
    for (int h = 1; h <= 12 && h * f < 19000.0; ++h)
        v += 0.12 / h * std::exp (-tn * 4.0 * (1.0 + 0.3 * h)) * std::sin (2 * juce::MathConstants<double>::pi * h * f * t + 0.7 * h);
    return (float) v;
}

int levels (const std::vector<juce::String>& irs)
{
    float (*input) (double) = guitar;
    auto render = [&input] (const juce::String& ir, double& rmsDb, double& kDb)
    {
        std::unique_ptr<juce::AudioProcessor> p (createPluginFilter());
        juce::AudioProcessor::BusesLayout layout;
        layout.inputBuses.add (juce::AudioChannelSet::mono());
        layout.outputBuses.add (juce::AudioChannelSet::stereo());
        p->setBusesLayout (layout);
        for (auto* prm : p->getParameters()) prm->setValueNotifyingHost (prm->getDefaultValue());
        constexpr int block = 256;
        p->setRateAndBufferSizeDetails (kRate, block);
        p->prepareToPlay (kRate, block);
        if (ir.isNotEmpty()) dynamic_cast<VibeAmpProcessor&> (*p).loadIR (juce::File (ir));
        juce::AudioBuffer<float> buf (2, block);
        juce::MidiBuffer midi;
        for (int k = 0; k < 6; ++k)
        {
            for (int b = 0; b < (int) (0.1 * kRate) / block; ++b) { buf.clear(); p->processBlock (buf, midi); }
            std::this_thread::sleep_for (std::chrono::milliseconds (150));
        }
        // K-weighting at 48 kHz (ITU-R BS.1770 coefficients)
        struct Bq
        {
            double b0, b1, b2, a1, a2, z1 = 0, z2 = 0;
            double processSample (double x) { const double y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
        };
        Bq f1 { 1.53512485958697, -2.69169618940638, 1.19839281085285, -1.69065929318241, 0.73248077421585 };
        Bq f2 { 1.0, -2.0, 1.0, -1.99004745483398, 0.99007225036621 };
        double s = 0, sk = 0;
        long n = 0;
        const int numBlocks = (int) (8.0 * kRate) / block;
        for (int b = 0; b < numBlocks; ++b)
        {
            for (int i = 0; i < block; ++i) { buf.setSample (0, i, input ((double) (b * block + i) / kRate)); buf.setSample (1, i, 0.0f); }
            p->processBlock (buf, midi);
            for (int i = 0; i < block; ++i)
            {
                const double y = buf.getSample (0, i), yk = f2.processSample (f1.processSample (y));
                if (b * block > (int) kRate) { s += y * y; sk += yk * yk; ++n; }
            }
        }
        rmsDb = 10 * std::log10 (s / n);
        kDb = 10 * std::log10 (sk / n);
    };
    std::printf ("default patch output level, dB re the built-in cab (plain RMS / K-weighted, as loudness meters)\n");
    std::printf ("%-48s %16s %16s\n", "", "chugs", "lead line");
    std::vector<double> base (4);
    input = guitar; render ({}, base[0], base[1]);
    input = leadLine; render ({}, base[2], base[3]);
    std::printf ("%-48s %+6.1f / %+6.1f  %+6.1f / %+6.1f\n", "built-in cab", 0.0, 0.0, 0.0, 0.0);
    for (const auto& ir : irs)
    {
        double r[4];
        input = guitar; render (ir, r[0], r[1]);
        input = leadLine; render (ir, r[2], r[3]);
        std::printf ("%-48s %+6.1f / %+6.1f  %+6.1f / %+6.1f\n", juce::File (ir).getFileNameWithoutExtension().toRawUTF8(),
                     r[0] - base[0], r[1] - base[1], r[2] - base[2], r[3] - base[3]);
    }
    return 0;
}

// plugin_cpu upgrade: a state saved before the 1176's DI (knob curves 3) loaded into the plugin: INPUT 5.56 lower,
// OUTPUT lowered along its taper where INPUT runs out (PresetManager::upgradeKnobCurves)
int upgrade()
{
    const double cases[][4] = { { 9.0, 8.5, 3.444, 8.5 }, { 7.0, 8.5, 1.444, 8.5 }, { 3.0, 9.0, 0.0, 6.76 } }; // old in/out, expected
    int bad = 0;
    for (const auto& c : cases)
    {
        std::unique_ptr<juce::AudioProcessor> p (createPluginFilter());
        juce::MemoryBlock mb;
        p->getStateInformation (mb);
        std::unique_ptr<juce::XmlElement> xml (juce::AudioProcessor::getXmlFromBinary (mb.getData(), (int) mb.getSize()));
        xml->setAttribute ("knobCurves", 3);
        for (auto* e : xml->getChildWithTagNameIterator ("PARAM"))
        {
            if (e->getStringAttribute ("id") == ids::compIn) e->setAttribute ("value", c[0]);
            if (e->getStringAttribute ("id") == ids::compOut) e->setAttribute ("value", c[1]);
        }
        juce::MemoryBlock old;
        juce::AudioProcessor::copyXmlToBinary (*xml, old);
        p->setStateInformation (old.getData(), (int) old.getSize());
        double in = -1, out = -1;
        for (auto* prm : p->getParameters())
            if (auto* r = dynamic_cast<juce::RangedAudioParameter*> (prm))
            {
                if (r->getParameterID() == ids::compIn) in = r->convertFrom0to1 (r->getValue());
                if (r->getParameterID() == ids::compOut) out = r->convertFrom0to1 (r->getValue());
            }
        const bool ok = std::abs (in - c[2]) < 0.01 && std::abs (out - c[3]) < 0.01;
        bad += ok ? 0 : 1;
        std::printf ("old INPUT %.2f OUTPUT %.2f -> %.3f / %.3f (expected %.3f / %.3f)  %s\n", c[0], c[1], in, out, c[2], c[3], ok ? "PASS" : "FAIL");
    }
    return bad;
}

int main (int argc, char** argv)
{
    juce::ScopedJuceInitialiser_GUI juce; // message manager for the parameter listeners
    std::vector<juce::String> irs;
    const bool levelsMode = argc > 1 && std::string (argv[1]) == "levels";
    for (int k = levelsMode ? 2 : 1; k < argc; ++k) irs.push_back (juce::String (juce::CharPointer_UTF8 (argv[k])));
    if (levelsMode) return levels (irs);
    if (argc > 1 && std::string (argv[1]) == "upgrade") return upgrade();
    const int blocks[] = { 64, 128, 256 };
    const juce::String mainIr = irs.empty() ? juce::String() : irs[0];

    std::printf ("== the whole plugin at 48 kHz (%% of one core; worst = slowest block as %% of its duration) ==\n");
    std::printf ("%-52s", "");
    for (int b : blocks) std::printf ("   %4d: avg / worst", b);
    std::printf ("\n");
    auto row = [&] (const std::string& name, const std::function<Result (int)>& fn)
    {
        std::printf ("%-52s", name.c_str());
        for (int b : blocks) { const auto r = fn (b); std::printf ("   %8.1f / %5.1f", r.avg, r.worst); }
        std::printf ("\n");
        std::fflush (stdout);
    };
    const Patch defaults = [] (juce::AudioProcessor&) {};
    row ("default patch, built-in cab", [&] (int b) { return runPlugin (defaults, {}, b); });
    for (const auto& ir : irs)
        row ("default patch, IR " + juce::File (ir).getFileNameWithoutExtension().substring (0, 30).toStdString(),
             [&] (int b) { return runPlugin (defaults, ir, b); });
    row ("everything on, built-in cab", [&] (int b) { return runPlugin (everythingOn, {}, b); });
    for (const auto& ir : irs)
        row ("everything on, IR " + juce::File (ir).getFileNameWithoutExtension().substring (0, 30).toStdString(),
             [&] (int b) { return runPlugin (everythingOn, ir, b); });

    std::printf ("\n== per stage: everything on%s, minus that stage (difference = its cost, %% of one core) ==\n",
                 mainIr.isEmpty() ? "" : (", IR " + juce::File (mainIr).getFileNameWithoutExtension()).toRawUTF8());
    struct Off { const char* name; const char* id; float value; };
    const Off offs[] = { { "1176", ids::compOn, 0 }, { "drive pedal", ids::boostOn, 0 }, { "3-band gate", ids::gateOn, 0 },
                         { "transpose LIVE -2", ids::transpose, 0 }, { "cab", ids::cabOn, 0 }, { "10-band EQ", ids::eqOn, 0 },
                         { "phaser", ids::phaserOn, 0 }, { "delay", ids::delayOn, 0 }, { "reverb", ids::reverbOn, 0 } };
    std::vector<Result> all;
    for (int b : blocks) all.push_back (runPlugin (everythingOn, mainIr, b));
    std::printf ("%-52s", "everything on");
    for (const auto& r : all) std::printf ("   %8.1f / %5.1f", r.avg, r.worst);
    std::printf ("\n");
    // each difference against an "everything on" run made right before it, so a slow drift (clock, other load)
    // affects both sides alike
    for (const auto& o : offs)
    {
        std::printf ("%-52s", (std::string ("  ") + o.name).c_str());
        for (size_t k = 0; k < 3; ++k)
        {
            const auto ref = runPlugin (everythingOn, mainIr, blocks[k]);
            const auto r = runPlugin ([&] (juce::AudioProcessor& p) { everythingOn (p); set (p, o.id, o.value); }, mainIr, blocks[k]);
            std::printf ("   %8.1f (%5.1f)  ", ref.avg - r.avg, ref.avg);
        }
        std::printf ("\n");
        std::fflush (stdout);
    }
    std::printf ("(in brackets: the paired everything-on run)\n");

    std::printf ("\n== inside: oversampling filters (module with its filters vs the circuits alone, %% of one core) ==\n");
    row ("AmpSim (8x JUCE IIR oversampling + circuits), RED", [&] (int b) {
        return runModule ([&] {
            auto a = std::make_shared<vamp::AmpSim>(); a->prepare (kRate, b);
            vamp::AmpSim::Settings s;
            return std::function<void (float*, int)> ([a, s] (float* d, int n) { a->process (d, n, s); });
        }, b); });
    row ("  circuits alone at 8x (preamp + power amp)", [&] (int b) {
        return runModule ([&] {
            auto pre = std::make_shared<vamp::Preamp5150>(); pre->setChannelTrims (54.0, 1.0, 1.0); pre->prepare (8 * kRate);
            pre->setModeNow (vamp::Preamp5150::Red); pre->setControls (0.6, 0.5, 0.5, 0.6, 0.3);
            auto pa = std::make_shared<vamp::PowerAmp5150>(); pa->setControls (0.5, 0.5); pa->prepare (8 * kRate);
            return std::function<void (float*, int)> ([pre, pa] (float* d, int n) {
                for (int i = 0; i < n; ++i)
                {
                    double y = 0;
                    for (int k = 0; k < 8; ++k) y = vamp::processAmp5150 (*pre, *pa, d[i], 1.0); // 8 samples per input sample
                    d[i] = (float) (y * vamp::kSpeakerToDigital);
                }
            });
        }, b); });
    row ("Comp1176 (2x oversampling + the 1176)", [&] (int b) {
        return runModule ([&] {
            auto c = std::make_shared<vamp::Comp1176>(); c->prepare (kRate, b);
            vamp::Comp1176::Settings s;
            return std::function<void (float*, int)> ([c, s] (float* d, int n) { c->process (d, n, s); });
        }, b); });
    return 0;
}
