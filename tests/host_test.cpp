// The whole plugin (VibeAmpProcessor) driven the way hosts and validators drive it, to catch crashes and
// out-of-range memory access in the processor itself (prepare / block-size / sample-rate handling):
//
//   host_test [auval | random | all] [seconds-per-case=2]
//
// auval   Apple auval's render sequence: mono in / stereo out, prepare + render at 44.1 kHz / 512,
//         22.05 kHz / 64, 96 kHz / 137, 48 kHz / 4096, 192 kHz / 4096, 11.025 kHz / 4096, 44.1 kHz / 512,
//         then mono / stereo layouts and blocks larger than prepared. Default and randomised parameters.
// random  random layouts, rates, prepared sizes, block sizes (often not the prepared one) and parameters.
// Every output sample must be finite and bounded (< 1e4: random settings stack up to ~+45 dBFS). Build with AddressSanitizer for the memory checks
// (cmake -DDD_ASAN=ON).
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <juce_audio_processors/juce_audio_processors.h>

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter();

namespace
{
std::mt19937 rng (1176);
int failures = 0;

double uniform (double lo, double hi) { return std::uniform_real_distribution<double> (lo, hi) (rng); }

void setDefaults (juce::AudioProcessor& p)
{
    for (auto* prm : p.getParameters()) prm->setValueNotifyingHost (prm->getDefaultValue());
}

void randomise (juce::AudioProcessor& p)
{
    for (auto* prm : p.getParameters())
    {
        const double r = uniform (0.0, 1.0); // extremes as often as values in between
        prm->setValueNotifyingHost (r < 0.2 ? 0.0f : r < 0.4 ? 1.0f : (float) uniform (0.0, 1.0));
    }
}

// prepare at (rate, maxBlock) with numIn / numOut channels, then render `seconds` of guitar-like input in
// blocks of `block` samples (block <= 0: random 1..maxBlock*2, as some hosts send larger blocks than announced)
bool render (juce::AudioProcessor& p, const char* name, double rate, int maxBlock, int block, int numIn, int numOut,
             double seconds)
{
    p.releaseResources();
    const auto set = [] (int n) { return n == 1 ? juce::AudioChannelSet::mono() : juce::AudioChannelSet::stereo(); };
    juce::AudioProcessor::BusesLayout layout;
    layout.inputBuses.add (set (numIn));
    layout.outputBuses.add (set (numOut));
    if (! p.setBusesLayout (layout)) { std::printf ("%-44s layout %d-%d refused\n", name, numIn, numOut); return true; }
    p.setRateAndBufferSizeDetails (rate, maxBlock);
    p.prepareToPlay (rate, maxBlock);

    const int chans = std::max (numIn, numOut);
    const int total = (int) (rate * seconds);
    juce::AudioBuffer<float> buf (chans, std::max (maxBlock * 2, std::max (block, 1)));
    juce::MidiBuffer midi;
    double phase = 0.0, peak = 0.0;
    bool finite = true;
    for (int done = 0; done < total;)
    {
        const int n = block > 0 ? block : std::uniform_int_distribution<int> (1, maxBlock * 2) (rng);
        buf.setSize (chans, n, false, false, true);
        for (int i = 0; i < n; ++i) // 110 Hz with a pick-like burst every 0.25 s, ~1 V peak
        {
            const double t = (done + i) / rate, env = std::exp (-8.0 * std::fmod (t, 0.25));
            phase += 2.0 * juce::MathConstants<double>::pi * 110.0 / rate;
            const float v = (float) (env * (0.8 * std::sin (phase) + 0.2 * uniform (-1.0, 1.0)));
            for (int c = 0; c < chans; ++c) buf.setSample (c, i, c < numIn ? v : 0.0f);
        }
        p.processBlock (buf, midi);
        for (int c = 0; c < numOut; ++c)
            for (int i = 0; i < n; ++i)
            {
                const float y = buf.getSample (c, i);
                finite = finite && std::isfinite (y);
                peak = std::max (peak, (double) std::abs (y));
            }
        done += n;
    }
    // every gain at its maximum (Output +24, Amp Volume +12, overlapping EQ bands, delay feedback 0.95) reaches
    // ~+45 dBFS legitimately; a runaway goes far past +80 dB or to inf
    const bool ok = finite && peak < 1e4;
    std::printf ("%-44s %6.0f Hz  max %5d  block %5d  %d-%d  peak %7.3f  %s\n", name, rate, maxBlock, block, numIn,
                 numOut, peak, ok ? "PASS" : "FAIL");
    if (! ok) ++failures;
    return ok;
}

void auvalSequence (juce::AudioProcessor& p, double seconds, const char* label)
{
    std::printf ("== auval render sequence (%s) ==\n", label);
    render (p, "render 512", 44100, 512, 512, 1, 2, seconds);
    render (p, "slicing render 64", 44100, 512, 64, 1, 2, seconds);
    const struct { double rate; int frames; } steps[] = { { 22050, 64 }, { 96000, 137 }, { 48000, 4096 },
                                                          { 192000, 4096 }, { 11025, 4096 }, { 44100, 512 } };
    for (const auto& s : steps) render (p, "render at rate", s.rate, s.frames, s.frames, 1, 2, seconds);
    render (p, "1 channel", 44100, 512, 512, 1, 1, seconds);
    render (p, "1 to 2 channels", 44100, 256, 256, 1, 2, seconds);
    render (p, "2 to 2 channels", 44100, 512, 512, 2, 2, seconds);
    render (p, "blocks larger than prepared", 48000, 256, 4096, 1, 2, seconds);
    render (p, "random blocks up to 2x prepared", 48000, 1024, 0, 1, 2, seconds);
}

void randomCases (juce::AudioProcessor& p, double seconds, int cases)
{
    std::printf ("== random hosts ==\n");
    const double rates[] = { 8000, 11025, 22050, 32000, 44100, 48000, 88200, 96000, 176400, 192000 };
    const int sizes[] = { 1, 7, 16, 32, 64, 128, 137, 256, 441, 512, 1024, 2048, 4096, 8192 };
    for (int k = 0; k < cases; ++k)
    {
        randomise (p);
        const double rate = rates[std::uniform_int_distribution<int> (0, 9) (rng)];
        const int maxBlock = sizes[std::uniform_int_distribution<int> (0, 13) (rng)];
        const int block = std::uniform_int_distribution<int> (0, 2) (rng) == 0 ? 0 : maxBlock;
        const int numIn = std::uniform_int_distribution<int> (1, 2) (rng), numOut = std::uniform_int_distribution<int> (1, 2) (rng);
        render (p, ("random case " + std::to_string (k + 1)).c_str(), rate, maxBlock, block, numIn, numOut, seconds);
    }
}
} // namespace

int main (int argc, char** argv)
{
    juce::ScopedJuceInitialiser_GUI juce; // message manager (the processor's AsyncUpdater, parameter listeners)
    const std::string mode = argc > 1 ? argv[1] : "all";
    const double seconds = argc > 2 ? std::atof (argv[2]) : 2.0;

    std::unique_ptr<juce::AudioProcessor> p (createPluginFilter());
    if (mode == "auval" || mode == "all")
    {
        setDefaults (*p);
        auvalSequence (*p, seconds, "default parameters");
        for (int k = 0; k < 3; ++k)
        {
            randomise (*p);
            auvalSequence (*p, seconds, ("random parameters " + std::to_string (k + 1)).c_str());
        }
    }
    if (mode == "random" || mode == "all")
        randomCases (*p, seconds * 0.5, 40);

    std::printf ("\n%d failed\n", failures);
    return failures == 0 ? 0 : 1;
}
