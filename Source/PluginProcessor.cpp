#include <juce_audio_formats/juce_audio_formats.h>
#include "PluginProcessor.h"
#include "PluginEditor.h"
#include "Parameters.h"
#include "dsp/CabMatch.h"

VibeAmpProcessor::VibeAmpProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput ("Input", juce::AudioChannelSet::mono(), true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "PARAMS", vamp::createParameterLayout()),
      presets (apvts, [this] (const juce::File& f) { loadIR (f); }, [this] { clearIR(); }, [this] { return getIRPath(); })
{
    auto get = [this] (const juce::String& id)
    {
        auto* p = apvts.getRawParameterValue (id);
        jassert (p != nullptr);
        return p;
    };

    pInGain = get (ids::inGain);
    pGateOn = get (ids::gateOn); pGateThresh = get (ids::gateThresh); pGateRelease = get (ids::gateRelease);
    pGateLow = get (ids::gateLow); pGateMid = get (ids::gateMid); pGateHigh = get (ids::gateHigh);
    pTranspose = get (ids::transpose); pTransposeMode = get (ids::transposeMode);
    pInGateOn = get (ids::inGateOn); pInGateThresh = get (ids::inGateThresh); pInGateRelease = get (ids::inGateRelease);
    pAmpVolume = get (ids::ampVolume);
    pCompOn = get (ids::compOn); pCompIn = get (ids::compIn); pCompOut = get (ids::compOut);
    pCompAtk = get (ids::compAtk); pCompRel = get (ids::compRel); pCompRatio = get (ids::compRatioSel);
    pBoostOn = get (ids::boostOn); pBoostDrive = get (ids::boostDrive); pBoostTone = get (ids::boostTone);
    pBoostLevel = get (ids::boostLevel); pBoostAttack = get (ids::boostAttack); pBoostModel = get (ids::boostModel);
    pChannel = get (ids::channel); pAmpBright = get (ids::ampBright); pAmpCrunch = get (ids::ampCrunch); pAmpNoise = get (ids::ampNoise); pTubeSet = get (ids::tubeSet); pAmpGain = get (ids::ampGain); pBass = get (ids::bass); pMid = get (ids::mid);
    pTreble = get (ids::treble); pPresence = get (ids::presence); pResonance = get (ids::resonance);
    pMaster = get (ids::master);
    pCabOn = get (ids::cabOn);
    pEqOn = get (ids::eqOn);
    for (int i = 0; i < 10; ++i)
        pEq[i] = get (ids::eqBand (i));
    pPhaserOn = get (ids::phaserOn); pPhRate = get (ids::phRate); pPhDepth = get (ids::phDepth);
    pPhFeedback = get (ids::phFeedback); pPhMix = get (ids::phMix);
    pDelayOn = get (ids::delayOn); pDlTime = get (ids::dlTime); pDlFeedback = get (ids::dlFeedback);
    pDlMix = get (ids::dlMix); pDlPingPong = get (ids::dlPingPong);
    pReverbOn = get (ids::reverbOn); pRvSize = get (ids::rvSize); pRvDamping = get (ids::rvDamping);
    pRvWidth = get (ids::rvWidth); pRvMix = get (ids::rvMix);
    pOutGain = get (ids::outGain);
}

bool VibeAmpProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto in = layouts.getMainInputChannelSet();
    const auto out = layouts.getMainOutputChannelSet();
    const auto ok = [] (const juce::AudioChannelSet& s)
    { return s == juce::AudioChannelSet::mono() || s == juce::AudioChannelSet::stereo(); };
    return ok (in) && ok (out);
}

void VibeAmpProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    sampleRateHz = sampleRate;
    maxBlock = juce::jmax (1, samplesPerBlock);

    monoBuf.setSize (1, maxBlock);
    keyBuf.setSize (1, maxBlock);
    stereoBuf.setSize (2, maxBlock);

    const juce::dsp::ProcessSpec monoSpec { sampleRate, (juce::uint32) maxBlock, 1 };
    const juce::dsp::ProcessSpec stereoSpec { sampleRate, (juce::uint32) maxBlock, 2 };

    inputGain.prepare (monoSpec);  inputGain.setRampDurationSeconds (0.05);
    outputGain.prepare (stereoSpec); outputGain.setRampDurationSeconds (0.05);

    gate.prepare (sampleRate);
    inputGate.prepare (sampleRate);

    shifter.prepare (sampleRate);
    comp1176.prepare (sampleRate, maxBlock);
    amp.prepare (sampleRate, maxBlock);
    cab.prepare (monoSpec);

    // Built-in fallback cabinet (used until the user loads an IR; loaded IRs are matched to its loudness)
    vamp::setupFallbackCab (fallbackCab, sampleRate);
    for (auto& f : eq) f.reset();
    std::fill (std::begin (eqGainApplied), std::end (eqGainApplied), std::numeric_limits<float>::quiet_NaN()); // coefficients again at this rate

    phaser.prepare (stereoSpec);
    phaser.setCentreFrequency (1000.0f);
    delay.prepare (sampleRate);
    reverb.prepare (stereoSpec);

    keyDelayBuf.assign (16384, 0.0f);
    keyDelayPos = 0;

    targetLatency = amp.getLatencySamples() + shifter.getReportedLatency() + (compOn ? comp1176.getLatencySamples() : 0);
    setLatencySamples (targetLatency.load());
}

void VibeAmpProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const int numIn = getTotalNumInputChannels();
    const int numOut = getTotalNumOutputChannels();
    const int numSamples = buffer.getNumSamples();

    // ---- read parameters once per block ----
    inputGain.setGainDecibels (pInGain->load());

    const bool newInGateOn = pInGateOn->load() > 0.5f;
    if (newInGateOn && ! inGateOn) inputGate.reset();
    inGateOn = newInGateOn;
    inGateThresh = pInGateThresh->load();
    inGateRelease = pInGateRelease->load();

    const bool newGateOn = pGateOn->load() > 0.5f;
    if (newGateOn && ! gateOn) gate.reset();
    gateOn = newGateOn;
    gateSettings.threshDb = pGateThresh->load();
    gateSettings.releaseMs = pGateRelease->load();
    gateSettings.depth[0] = pGateLow->load() * 0.01f;
    gateSettings.depth[1] = pGateMid->load() * 0.01f;
    gateSettings.depth[2] = pGateHigh->load() * 0.01f;

    shifter.setParams (juce::roundToInt (pTranspose->load()), juce::roundToInt (pTransposeMode->load()));

    // UREI 1176LN
    const bool newCompOn = pCompOn->load() > 0.5f;
    if (newCompOn && ! compOn) comp1176.reset(); // switched in: starts from its settled idle point
    compOn = newCompOn;
    compSettings.input = pCompIn->load();
    compSettings.output = pCompOut->load();
    compSettings.attack = pCompAtk->load();
    compSettings.release = pCompRel->load();
    compSettings.ratio = juce::roundToInt (pCompRatio->load());
    if (! compOn) compGr.store (0.0f);
    const int compLatency = compOn ? comp1176.getLatencySamples() : 0;
    keyDelay = juce::jlimit (0, (int) keyDelayBuf.size() - 1,
                             (juce::roundToInt (pTransposeMode->load()) == vamp::PitchShifter::HQ ? shifter.getAverageDelay() : 0)
                                 + compLatency);

    // HQ transpose and the compressor's oversampling add latency: report it to the host (on the message thread)
    const int latency = amp.getLatencySamples() + shifter.getReportedLatency() + compLatency;
    if (latency != targetLatency.load())
    {
        targetLatency = latency;
        triggerAsyncUpdate();
    }

    ampSettings.channel = juce::roundToInt (pChannel->load());
    ampSettings.bright = pAmpBright->load() > 0.5f;
    ampSettings.crunch = pAmpCrunch->load() > 0.5f;
    ampSettings.noise = pAmpNoise->load() > 0.5f;
    ampSettings.tubeSet = juce::roundToInt (pTubeSet->load());
    ampSettings.gain = pAmpGain->load() * 0.1f;
    ampSettings.bass = pBass->load() * 0.1f;
    ampSettings.mid = pMid->load() * 0.1f;
    ampSettings.treble = pTreble->load() * 0.1f;
    ampSettings.presence = pPresence->load() * 0.1f;
    ampSettings.resonance = pResonance->load() * 0.1f;
    ampSettings.master = pMaster->load() * 0.1f;
    ampSettings.volumeDb = pAmpVolume->load();
    ampSettings.boostOn = pBoostOn->load() > 0.5f;
    ampSettings.boostDrive = pBoostDrive->load() * 0.1f;
    ampSettings.boostBright = pBoostTone->load() * 0.1f;
    ampSettings.boostLevel = pBoostLevel->load() * 0.1f;
    ampSettings.boostAttack = juce::roundToInt (pBoostAttack->load()) - 1;
    ampSettings.boostModel = juce::roundToInt (pBoostModel->load());

    cabOn = pCabOn->load() > 0.5f;

    eqOn = pEqOn->load() > 0.5f;
    if (eqOn) // coefficients only for a band whose gain moved (every block, they cost more than the filters at small buffers)
        for (int i = 0; i < 10; ++i)
        {
            const float g = pEq[i]->load();
            if (g == eqGainApplied[i]) continue;
            eqGainApplied[i] = g;
            eq[i].set (vamp::Biquad::Type::peak, sampleRateHz, ids::eqFreqs[i], 1.41, g);
        }

    phaserOn = pPhaserOn->load() > 0.5f;
    phaser.setRate (pPhRate->load());
    phaser.setDepth (pPhDepth->load());
    phaser.setFeedback (pPhFeedback->load());
    phaser.setMix (pPhMix->load());

    delayOn = pDelayOn->load() > 0.5f;
    dlTime = pDlTime->load();
    dlFeedback = pDlFeedback->load();
    dlMix = pDlMix->load();
    pingPong = pDlPingPong->load() > 0.5f;

    reverbOn = pReverbOn->load() > 0.5f;
    juce::Reverb::Parameters rp;
    rp.roomSize = pRvSize->load();
    rp.damping = pRvDamping->load();
    rp.width = pRvWidth->load();
    rp.wetLevel = pRvMix->load() * 0.5f;
    rp.dryLevel = 0.5f; // JUCE scales dry by 2, so 0.5 == unity
    rp.freezeMode = 0.0f;
    reverb.setParameters (rp);

    outputGain.setGainDecibels (pOutGain->load());

    // clear tails when an effect gets switched back on
    if (delayOn && ! wasDelayOn) delay.reset();
    if (reverbOn && ! wasReverbOn) reverb.reset();
    if (phaserOn && ! wasPhaserOn) phaser.reset();
    wasDelayOn = delayOn; wasReverbOn = reverbOn; wasPhaserOn = phaserOn;

    // ---- process in chunks no larger than what we prepared for ----
    for (int start = 0; start < numSamples; start += maxBlock)
        processChunk (buffer, start, juce::jmin (maxBlock, numSamples - start), numIn, numOut);
}

void VibeAmpProcessor::processChunk (juce::AudioBuffer<float>& buffer, int start, int len, int numIn, int numOut)
{
    // Guitar is mono: take the first input channel
    if (numIn > 0)
        monoBuf.copyFrom (0, 0, buffer, 0, start, len);
    else
        monoBuf.clear (0, 0, len);

    float* mono = monoBuf.getWritePointer (0);
    // A non-finite input sample (broken upstream plugin / host) would poison every stateful block after it
    // (pitch shifter, compressor, oversampling filters, cab, delay, reverb) until the plugin is reloaded.
    // It is treated as lost: the last finite sample stands in.
    for (int i = 0; i < len; ++i)
    {
        if (std::isfinite (mono[i])) lastInput = mono[i];
        else mono[i] = lastInput;
    }
    auto monoBlock = juce::dsp::AudioBlock<float> (monoBuf).getSubBlock (0, (size_t) len);
    juce::dsp::ProcessContextReplacing<float> monoCtx (monoBlock);

    // ---- pre-amp: input -> transpose -> comp ----
    inputGain.process (monoCtx);
    updatePeak (inputPeak, monoBuf.getMagnitude (0, 0, len)); // what the amp sees (after the Input knob)
    {
        // clean guitar = noise gate detector key, delayed to line up with the transposed signal
        float* key = keyBuf.getWritePointer (0);
        const int mask = (int) keyDelayBuf.size() - 1;
        for (int i = 0; i < len; ++i)
        {
            keyDelayBuf[(size_t) keyDelayPos] = mono[i];
            key[i] = keyDelayBuf[(size_t) ((keyDelayPos - keyDelay) & mask)];
            keyDelayPos = (keyDelayPos + 1) & mask;
        }
    }
    if (inGateOn)
        inputGate.process (mono, len, inGateThresh, inGateRelease);
    shifter.process (mono, len);
    if (compOn)
    {
        comp1176.process (mono, len, compSettings);
        compGr.store (comp1176.getGainReductionDb());
    }

    // ---- amp (boost lives inside the oversampled amp loop) ----
    amp.process (mono, len, ampSettings);

    // ---- 3-band noise gate: keyed by the clean input, acting on the amp output ----
    if (gateOn)
    {
        gate.process (mono, keyBuf.getReadPointer (0), len, gateSettings);
        for (int b = 0; b < 3; ++b)
            gateBandGain[b].store (gate.getBandGain (b));
    }
    else
    {
        for (auto& g : gateBandGain) g.store (1.0f);
    }

    // ---- cabinet ----
    takeIR();
    if (cabOn)
    {
        if (irLoaded.load())
            cab.process (monoCtx);
        else
            for (int i = 0; i < len; ++i)
            {
                float v = mono[i];
                for (auto& f : fallbackCab) v = f.process (v);
                mono[i] = v;
            }
    }

    // ---- 10-band EQ ----
    if (eqOn)
        for (int i = 0; i < len; ++i)
        {
            float v = mono[i];
            for (auto& f : eq) v = f.process (v);
            mono[i] = v;
        }

    // ---- stereo post effects ----
    stereoBuf.copyFrom (0, 0, monoBuf, 0, 0, len);
    stereoBuf.copyFrom (1, 0, monoBuf, 0, 0, len);
    auto stereoBlock = juce::dsp::AudioBlock<float> (stereoBuf).getSubBlock (0, (size_t) len);
    juce::dsp::ProcessContextReplacing<float> stereoCtx (stereoBlock);

    if (phaserOn) phaser.process (stereoCtx);
    if (delayOn) delay.process (stereoBuf.getWritePointer (0), stereoBuf.getWritePointer (1), len,
                                dlTime, dlFeedback, dlMix, pingPong);
    if (reverbOn) reverb.process (stereoCtx);
    outputGain.process (stereoCtx);
    updatePeak (outputPeak, juce::jmax (stereoBuf.getMagnitude (0, 0, len), stereoBuf.getMagnitude (1, 0, len)));

    // ---- write out ----
    if (numOut >= 2)
    {
        buffer.copyFrom (0, start, stereoBuf, 0, 0, len);
        buffer.copyFrom (1, start, stereoBuf, 1, 0, len);
    }
    else if (numOut == 1)
    {
        buffer.copyFrom (0, start, stereoBuf, 0, 0, len);
        buffer.addFrom (0, start, stereoBuf, 1, 0, len);
        buffer.applyGain (0, start, len, 0.5f);
    }

    for (int ch = juce::jlimit (0, 2, numOut); ch < buffer.getNumChannels(); ++ch)
        buffer.clear (ch, start, len);
}

// ---------------------------------------------------------------------------
// IR handling
// ---------------------------------------------------------------------------
void VibeAmpProcessor::loadIR (const juce::File& file)
{
    if (! file.existsAsFile())
        return;

    // The IR's first channel, scaled to the built-in cab's loudness (vamp::cabMatchGain: the amp's spectrum, K-weighted), so
    // the level does not jump between the built-in cab and IRs or from one IR to another; the tone is untouched.
    // (JUCE's own normalisation sets every IR to the same energy, which left IRs ~5 dB below the built-in cab and
    // differing from one another with where their energy lies.)
    juce::AudioFormatManager formats;
    formats.registerBasicFormats();
    std::unique_ptr<juce::AudioFormatReader> reader (formats.createReaderFor (file));
    if (reader == nullptr || reader->lengthInSamples <= 0 || reader->sampleRate <= 0.0)
        return;
    const int len = (int) std::min<juce::int64> (reader->lengthInSamples, (juce::int64) (10.0 * reader->sampleRate));
    juce::AudioBuffer<float> ir (1, len);
    reader->read (&ir, 0, len, 0, true, false);
    const double irRate = reader->sampleRate;
    ir.applyGain ((float) vamp::cabMatchGain (ir.getReadPointer (0), len, irRate));
    {
        // to the audio thread (takeIR): a previous IR it has not taken yet is replaced (and freed here)
        const juce::SpinLock::ScopedLockType lock (irPendingLock);
        irPending = std::move (ir);
        irPendingRate = irRate;
        irPendingSet = true;
    }
    {
        const juce::SpinLock::ScopedLockType lock (irLock);
        irPath = file.getFullPathName();
    }
}

void VibeAmpProcessor::clearIR()
{
    {
        const juce::SpinLock::ScopedLockType lock (irPendingLock);
        irPendingSet = false; // an IR still on its way is dropped
        irLoaded = false;
    }
    const juce::SpinLock::ScopedLockType lock (irLock);
    irPath.clear();
}

// audio thread: hand a newly loaded IR to cab (wait-free: the buffer moves into JUCE's loader, which builds the
// engine on its own thread and crossfades to it)
void VibeAmpProcessor::takeIR()
{
    const juce::SpinLock::ScopedTryLockType lock (irPendingLock);
    if (! lock.isLocked() || ! irPendingSet)
        return;
    irPendingSet = false;
    // Normalise::no: JUCE only rescales for a sample-rate change, which keeps the frequency response
    cab.loadImpulseResponse (std::move (irPending), irPendingRate, juce::dsp::Convolution::Stereo::no,
                             juce::dsp::Convolution::Trim::yes, juce::dsp::Convolution::Normalise::no);
    irLoaded = true;
}

void VibeAmpProcessor::stepIR (int direction)
{
    const auto current = getIRPath();
    if (current.isEmpty() || ! juce::File::isAbsolutePath (current))
        return;

    const juce::File cur (current);
    auto files = cur.getParentDirectory().findChildFiles (juce::File::findFiles, false, "*.wav;*.aif;*.aiff");
    if (files.isEmpty())
        return;

    files.sort();
    int idx = files.indexOf (cur);
    if (idx < 0) idx = 0;
    idx = (idx + direction + files.size()) % files.size();
    loadIR (files[idx]);
}

juce::String VibeAmpProcessor::getIRPath() const
{
    const juce::SpinLock::ScopedLockType lock (irLock);
    return irPath;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
void VibeAmpProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    auto state = apvts.copyState();
    state.setProperty ("irPath", getIRPath(), nullptr);
    state.setProperty ("presetName", presets.getCurrentName(), nullptr);
    state.setProperty (kKnobCurvesAttr, kKnobCurvesVersion, nullptr);
    if (auto xml = state.createXml())
        copyXmlToBinary (*xml, destData);
}

void VibeAmpProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    std::unique_ptr<juce::XmlElement> xml (getXmlFromBinary (data, sizeInBytes));
    if (xml == nullptr || ! xml->hasTagName (apvts.state.getType()))
        return;

    upgradeKnobCurves (*xml); // v0.3 sessions keep their sound
    auto tree = juce::ValueTree::fromXml (*xml);
    apvts.replaceState (tree);

    presets.setCurrentName (tree.getProperty ("presetName", juce::String()).toString()); // empty -> "(no preset)"

    const auto path = tree.getProperty ("irPath").toString();
    if (path.isNotEmpty() && juce::File::isAbsolutePath (path) && juce::File (path).existsAsFile())
        loadIR (juce::File (path));
    else
        clearIR();
}

juce::AudioProcessorEditor* VibeAmpProcessor::createEditor()
{
    return new VibeAmpEditor (*this);
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new VibeAmpProcessor();
}
