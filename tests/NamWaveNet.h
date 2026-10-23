#pragma once
// Standalone NAM WaveNet inference (standard A1 .nam files), shared by the comparison tools.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace namcmp
{
// ---------------------------------------------------------------- NAM WaveNet inference
// Layout and weight order of NeuralAmpModelerCore (wavenet.cpp): per layer array a 1x1 rechannel
// (no bias), per layer a dilated conv (bias) + input mixin (1x1, no bias) + 1x1 (bias), then a 1x1
// head rechannel (bias = head_bias); the last weight is head_scale. Conv weights are stored
// out-channel, in-channel, tap; tap K-1 is the current sample, tap k is delayed by d (K-1-k).
class NamWaveNet
{
public:
    bool load (const std::string& path)
    {
        std::ifstream f (path, std::ios::binary);
        if (! f) return false;
        std::stringstream ss; ss << f.rdbuf();
        const std::string js = ss.str();

        sampleRate = number (js, "\"sample_rate\"", 48000.0);
        {
            const size_t k = js.find ("\"name\"");
            if (k != std::string::npos)
            {
                const size_t q0 = js.find ('"', js.find (':', k) + 1), q1 = js.find ('"', q0 + 1);
                name = js.substr (q0 + 1, q1 - q0 - 1);
            }
        }
        // layer array configs, in order
        size_t pos = js.find ("\"layers\"");
        if (pos == std::string::npos) return false;
        while (true)
        {
            const size_t in = js.find ("\"input_size\"", pos);
            const size_t wpos = js.find ("\"weights\"");
            if (in == std::string::npos || in > wpos) break;
            Array a;
            a.inputSize = (int) number (js.substr (in), "\"input_size\"");
            a.conditionSize = (int) number (js.substr (in), "\"condition_size\"");
            a.headSize = (int) number (js.substr (in), "\"head_size\"");
            a.channels = (int) number (js.substr (in), "\"channels\"");
            a.kernel = (int) number (js.substr (in), "\"kernel_size\"");
            a.headBias = js.substr (in, js.find ('}', in) - in).find ("\"head_bias\": true") != std::string::npos;
            const std::string gated = js.substr (in, js.find ('}', in) - in);
            if (gated.find ("\"gated\": true") != std::string::npos) { std::printf ("gated WaveNet not supported\n"); return false; }
            if (gated.find ("\"activation\": \"Tanh\"") == std::string::npos) { std::printf ("only Tanh supported\n"); return false; }
            const size_t dl = js.find ('[', js.find ("\"dilations\"", in));
            const size_t dr = js.find (']', dl);
            std::stringstream ds (js.substr (dl + 1, dr - dl - 1));
            std::string tok;
            while (std::getline (ds, tok, ',')) a.dilations.push_back (std::stoi (tok));
            arrays.push_back (a);
            pos = dr;
        }

        // weights
        const size_t wl = js.find ('[', js.find ("\"weights\""));
        const size_t wr = js.find (']', wl);
        std::vector<double> w;
        {
            const char* p = js.c_str() + wl + 1;
            const char* end = js.c_str() + wr;
            while (p < end)
            {
                char* next = nullptr;
                const double v = std::strtod (p, &next);
                if (next == p) { ++p; continue; }
                w.push_back (v);
                p = next;
            }
        }

        size_t it = 0;
        auto take = [&] (size_t n) { std::vector<double> v (w.begin() + (long) it, w.begin() + (long) (it + n)); it += n; return v; };
        for (auto& a : arrays)
        {
            a.rechannel = take ((size_t) (a.channels * a.inputSize));
            for (int d : a.dilations)
            {
                Layer l;
                l.dilation = d;
                l.conv = take ((size_t) (a.channels * a.channels * a.kernel));
                l.convBias = take ((size_t) a.channels);
                l.mixin = take ((size_t) (a.channels * a.conditionSize));
                l.oneByOne = take ((size_t) (a.channels * a.channels));
                l.oneByOneBias = take ((size_t) a.channels);
                l.histLen = d * (a.kernel - 1) + 1;
                l.history.assign ((size_t) (a.channels * l.histLen), 0.0);
                a.layers.push_back (std::move (l));
            }
            a.headRechannel = take ((size_t) (a.headSize * a.channels));
            if (a.headBias) a.headRechannelBias = take ((size_t) a.headSize);
        }
        headScale = w[it++];
        if (it != w.size()) { std::printf ("weight count mismatch: used %zu of %zu\n", it, w.size()); return false; }
        return true;
    }

    double getSampleRate() const { return sampleRate; }
    const std::string& getName() const { return name; }

    float process (float x)
    {
        std::vector<double> in { (double) x };
        std::vector<double> head; // head accumulator handed from array to array
        for (size_t ai = 0; ai < arrays.size(); ++ai)
        {
            auto& a = arrays[ai];
            const int C = a.channels;
            std::vector<double> z ((size_t) C, 0.0);
            for (int o = 0; o < C; ++o)
                for (int i = 0; i < a.inputSize; ++i)
                    z[(size_t) o] += a.rechannel[(size_t) (o * a.inputSize + i)] * in[(size_t) i];
            if (head.empty()) head.assign ((size_t) C, 0.0);

            for (auto& l : a.layers)
            {
                // store the layer input in its history ring
                l.pos = (l.pos + 1) % l.histLen;
                for (int c = 0; c < C; ++c) l.history[(size_t) (l.pos * C + c)] = z[(size_t) c];

                std::vector<double> y (l.convBias);
                for (int k = 0; k < a.kernel; ++k)
                {
                    const int delay = l.dilation * (a.kernel - 1 - k);
                    const int idx = (l.pos - delay + l.histLen * 4) % l.histLen;
                    const double* h = &l.history[(size_t) (idx * C)];
                    for (int o = 0; o < C; ++o)
                    {
                        double s = 0.0;
                        const double* wrow = &l.conv[(size_t) (o * C * a.kernel)];
                        for (int i = 0; i < C; ++i) s += wrow[i * a.kernel + k] * h[i];
                        y[(size_t) o] += s;
                    }
                }
                for (int o = 0; o < C; ++o)
                {
                    y[(size_t) o] += l.mixin[(size_t) o] * x; // condition = the model input
                    y[(size_t) o] = std::tanh (y[(size_t) o]);
                    head[(size_t) o] += y[(size_t) o];
                }
                for (int o = 0; o < C; ++o)
                {
                    double s = l.oneByOneBias[(size_t) o];
                    for (int i = 0; i < C; ++i) s += l.oneByOne[(size_t) (o * C + i)] * y[(size_t) i];
                    z[(size_t) o] += s;
                }
            }

            std::vector<double> h2 ((size_t) a.headSize, 0.0);
            for (int o = 0; o < a.headSize; ++o)
            {
                double s = a.headBias ? a.headRechannelBias[(size_t) o] : 0.0;
                for (int i = 0; i < C; ++i) s += a.headRechannel[(size_t) (o * C + i)] * head[(size_t) i];
                h2[(size_t) o] = s;
            }
            head = h2;
            in = z;
        }
        return (float) (head[0] * headScale);
    }

private:
    struct Layer
    {
        int dilation = 1, histLen = 1, pos = 0;
        std::vector<double> conv, convBias, mixin, oneByOne, oneByOneBias, history;
    };
    struct Array
    {
        int inputSize = 1, conditionSize = 1, headSize = 1, channels = 1, kernel = 3;
        bool headBias = false;
        std::vector<int> dilations;
        std::vector<double> rechannel, headRechannel, headRechannelBias;
        std::vector<Layer> layers;
    };

    static double number (const std::string& js, const char* key, double def = 0.0)
    {
        const size_t k = js.find (key);
        if (k == std::string::npos) return def;
        return std::strtod (js.c_str() + js.find (':', k) + 1, nullptr);
    }

    std::vector<Array> arrays;
    double headScale = 1.0, sampleRate = 48000.0;
    std::string name;
};
} // namespace namcmp
