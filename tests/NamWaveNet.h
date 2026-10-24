#pragma once
// Standalone NAM WaveNet inference, shared by the comparison tools. Reads standard A1 files (version 0.5.x) and the
// A2 WaveNets of NAM 0.7 (also inside a SlimmableContainer, whose full-size submodel is used), as far as the
// captures used here need: per-layer kernel sizes, Tanh / LeakyReLU / ReLU, a bottleneck with the layer 1x1, and a
// head conv of any kernel. Gating, grouped convs, head 1x1, FiLM and post-stack heads are rejected.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace namcmp
{
// ---------------------------------------------------------------- minimal JSON
struct Json
{
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::vector<Json> arr;
    std::vector<std::pair<std::string, Json>> obj;

    const Json* get (const std::string& key) const
    {
        for (const auto& kv : obj) if (kv.first == key) return &kv.second;
        return nullptr;
    }
    double numOr (const std::string& key, double def) const { const Json* j = get (key); return j && j->type == Number ? j->num : def; }
    bool boolOr (const std::string& key, bool def) const { const Json* j = get (key); return j && j->type == Bool ? j->b : def; }

    static bool parse (const char*& p, const char* end, Json& out)
    {
        auto ws = [&] { while (p < end && (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t')) ++p; };
        ws();
        if (p >= end) return false;
        if (*p == '{')
        {
            out.type = Object; ++p; ws();
            if (p < end && *p == '}') { ++p; return true; }
            while (p < end)
            {
                Json key;
                if (! parse (p, end, key) || key.type != String) return false;
                ws(); if (p >= end || *p != ':') return false; ++p;
                out.obj.emplace_back (key.str, Json());
                if (! parse (p, end, out.obj.back().second)) return false;
                ws();
                if (p < end && *p == ',') { ++p; continue; }
                if (p < end && *p == '}') { ++p; return true; }
                return false;
            }
            return false;
        }
        if (*p == '[')
        {
            out.type = Array; ++p; ws();
            if (p < end && *p == ']') { ++p; return true; }
            while (p < end)
            {
                out.arr.emplace_back();
                if (! parse (p, end, out.arr.back())) return false;
                ws();
                if (p < end && *p == ',') { ++p; continue; }
                if (p < end && *p == ']') { ++p; return true; }
                return false;
            }
            return false;
        }
        if (*p == '"')
        {
            out.type = String; ++p;
            while (p < end && *p != '"') { if (*p == '\\' && p + 1 < end) ++p; out.str += *p++; }
            if (p >= end) return false;
            ++p;
            return true;
        }
        if (end - p >= 4 && std::string (p, 4) == "true") { out.type = Bool; out.b = true; p += 4; return true; }
        if (end - p >= 5 && std::string (p, 5) == "false") { out.type = Bool; out.b = false; p += 5; return true; }
        if (end - p >= 4 && std::string (p, 4) == "null") { out.type = Null; p += 4; return true; }
        char* next = nullptr;
        out.num = std::strtod (p, &next);
        if (next == p) return false;
        out.type = Number;
        p = next;
        return true;
    }
};

// ---------------------------------------------------------------- NAM WaveNet inference
// Layout and weight order of NeuralAmpModelerCore (wavenet/model.cpp): per layer array a 1x1 rechannel (no bias),
// per layer a dilated conv channels -> bottleneck (bias) + input mixin condition -> bottleneck (1x1, no bias), the
// activation, the layer 1x1 bottleneck -> channels (bias; the residual), the activated conv output summed into the
// head; then the head conv bottleneck -> head size (kernel / dilation / bias from the config; A1: a 1x1); the last
// weight is head_scale. Conv weights are stored out-channel, in-channel, tap; tap k is delayed by d (K-1-k).
class NamWaveNet
{
public:
    bool load (const std::string& path)
    {
        std::ifstream f (path, std::ios::binary);
        if (! f) return false;
        std::stringstream ss; ss << f.rdbuf();
        const std::string text = ss.str();
        const char* p = text.c_str();
        Json root;
        if (! Json::parse (p, p + text.size(), root) || root.type != Json::Object) { std::printf ("not a NAM JSON file\n"); return false; }

        if (const Json* md = root.get ("metadata"))
            if (const Json* n = md->get ("name")) name = n->str;

        const Json* model = &root;
        if (const Json* a = root.get ("architecture"); a && a->str == "SlimmableContainer")
        {
            // the submodel with the largest max_value: the full-size network
            const Json* subs = root.get ("config") ? root.get ("config")->get ("submodels") : nullptr;
            if (! subs || subs->arr.empty()) { std::printf ("SlimmableContainer without submodels\n"); return false; }
            const Json* best = nullptr; double bestMax = -1.0;
            for (const auto& s : subs->arr)
                if (s.numOr ("max_value", 0.0) > bestMax) { bestMax = s.numOr ("max_value", 0.0); best = s.get ("model"); }
            if (! best) return false;
            model = best;
        }
        const Json* arch = model->get ("architecture");
        if (! arch || arch->str != "WaveNet") { std::printf ("only WaveNet models are supported\n"); return false; }
        sampleRate = model->numOr ("sample_rate", root.numOr ("sample_rate", 48000.0));
        const Json* cfg = model->get ("config");
        const Json* wj = model->get ("weights");
        if (! cfg || ! wj) return false;
        if (const Json* h = cfg->get ("head"); h && h->type != Json::Null) { std::printf ("post-stack head not supported\n"); return false; }

        for (const auto& lj : cfg->get ("layers")->arr)
        {
            Array a;
            a.inputSize = (int) lj.numOr ("input_size", 1);
            a.conditionSize = (int) lj.numOr ("condition_size", 1);
            a.channels = (int) lj.numOr ("channels", 1);
            a.bottleneck = (int) lj.numOr ("bottleneck", a.channels);
            if (const Json* hj = lj.get ("head"); hj && hj->type == Json::Object) // A2
            {
                a.headSize = (int) hj->numOr ("out_channels", 1);
                a.headKernel = (int) hj->numOr ("kernel_size", 1);
                a.headDilation = (int) hj->numOr ("dilation", 1);
                a.headBias = hj->boolOr ("bias", false);
            }
            else // A1
            {
                a.headSize = (int) lj.numOr ("head_size", 1);
                a.headBias = lj.boolOr ("head_bias", false);
            }
            for (const auto& d : lj.get ("dilations")->arr) a.dilations.push_back ((int) d.num);
            const size_t n = a.dilations.size();
            if (const Json* ks = lj.get ("kernel_sizes"); ks && ks->type == Json::Array)
                for (const auto& k : ks->arr) a.kernels.push_back ((int) k.num);
            else
                a.kernels.assign (n, (int) lj.numOr ("kernel_size", 3));
            if (a.kernels.size() != n) { std::printf ("kernel_sizes / dilations mismatch\n"); return false; }

            // activation: one name for all layers (A1) or one object per layer (A2)
            const Json* act = lj.get ("activation");
            for (size_t i = 0; i < n; ++i)
            {
                const Json* aj = act && act->type == Json::Array ? &act->arr[i] : act;
                Act ac;
                const std::string type = ! aj ? "Tanh" : aj->type == Json::String ? aj->str : aj->get ("type") ? aj->get ("type")->str : "";
                if (type == "Tanh") ac.kind = Act::Tanh;
                else if (type == "ReLU") ac.kind = Act::Leaky, ac.slope = 0.0;
                else if (type == "LeakyReLU") ac.kind = Act::Leaky, ac.slope = aj->type == Json::Object ? aj->numOr ("negative_slope", 0.01) : 0.01;
                else { std::printf ("activation %s not supported\n", type.c_str()); return false; }
                a.acts.push_back (ac);
            }

            // features this implementation does not do
            if (lj.boolOr ("gated", false)) { std::printf ("gated WaveNet not supported\n"); return false; }
            if (const Json* gm = lj.get ("gating_mode"); gm && gm->type == Json::Array)
                for (const auto& g : gm->arr) if (g.type == Json::String && g.str != "none") { std::printf ("gating not supported\n"); return false; }
            if (lj.numOr ("groups_input", 1) != 1 || lj.numOr ("groups_input_mixin", 1) != 1) { std::printf ("grouped convs not supported\n"); return false; }
            if (const Json* h1 = lj.get ("head1x1"); h1 && h1->boolOr ("active", false)) { std::printf ("head1x1 not supported\n"); return false; }
            if (const Json* l1 = lj.get ("layer1x1"))
            {
                if (! l1->boolOr ("active", true) || l1->numOr ("groups", 1) != 1) { std::printf ("layer1x1 inactive / grouped not supported\n"); return false; }
            }
            for (const auto& kv : lj.obj)
                if (kv.first.find ("film") != std::string::npos && kv.second.boolOr ("active", false)) { std::printf ("FiLM not supported\n"); return false; }
            arrays.push_back (a);
        }

        std::vector<double> w;
        w.reserve (wj->arr.size());
        for (const auto& v : wj->arr) w.push_back (v.num);
        size_t it = 0;
        auto take = [&] (size_t n) { std::vector<double> v (w.begin() + (long) it, w.begin() + (long) (it + n)); it += n; return v; };
        for (auto& a : arrays)
        {
            const size_t C = (size_t) a.channels, B = (size_t) a.bottleneck;
            a.rechannel = take (C * (size_t) a.inputSize);
            for (size_t li = 0; li < a.dilations.size(); ++li)
            {
                Layer l;
                l.dilation = a.dilations[li];
                l.kernel = a.kernels[li];
                l.act = a.acts[li];
                l.conv = take (B * C * (size_t) l.kernel);
                l.convBias = take (B);
                l.mixin = take (B * (size_t) a.conditionSize);
                l.oneByOne = take (C * B);
                l.oneByOneBias = take (C);
                l.histLen = l.dilation * (l.kernel - 1) + 1;
                l.history.assign (C * (size_t) l.histLen, 0.0);
                a.layers.push_back (std::move (l));
            }
            a.headConv = take ((size_t) a.headSize * B * (size_t) a.headKernel);
            if (a.headBias) a.headConvBias = take ((size_t) a.headSize);
            a.headHistLen = a.headDilation * (a.headKernel - 1) + 1;
            a.headHistory.assign (B * (size_t) a.headHistLen, 0.0);
        }
        if (it >= w.size()) { std::printf ("weight count mismatch: model needs more than %zu\n", w.size()); return false; }
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
        for (auto& a : arrays)
        {
            const int C = a.channels, B = a.bottleneck;
            std::vector<double> z ((size_t) C, 0.0);
            for (int o = 0; o < C; ++o)
                for (int i = 0; i < a.inputSize; ++i)
                    z[(size_t) o] += a.rechannel[(size_t) (o * a.inputSize + i)] * in[(size_t) i];
            if (head.size() != (size_t) B) head.assign ((size_t) B, 0.0);

            for (auto& l : a.layers)
            {
                // store the layer input in its history ring
                l.pos = (l.pos + 1) % l.histLen;
                for (int c = 0; c < C; ++c) l.history[(size_t) (l.pos * C + c)] = z[(size_t) c];

                std::vector<double> y (l.convBias);
                for (int k = 0; k < l.kernel; ++k)
                {
                    const int delay = l.dilation * (l.kernel - 1 - k);
                    const int idx = ((l.pos - delay) % l.histLen + l.histLen) % l.histLen;
                    const double* h = &l.history[(size_t) (idx * C)];
                    for (int o = 0; o < B; ++o)
                    {
                        double s = 0.0;
                        const double* wrow = &l.conv[(size_t) (o * C * l.kernel)];
                        for (int i = 0; i < C; ++i) s += wrow[i * l.kernel + k] * h[i];
                        y[(size_t) o] += s;
                    }
                }
                for (int o = 0; o < B; ++o)
                {
                    double v = y[(size_t) o] + l.mixin[(size_t) o] * x; // condition = the model input
                    v = l.act.kind == Act::Tanh ? std::tanh (v) : (v > 0.0 ? v : l.act.slope * v);
                    y[(size_t) o] = v;
                    head[(size_t) o] += v;
                }
                for (int o = 0; o < C; ++o)
                {
                    double s = l.oneByOneBias[(size_t) o];
                    for (int i = 0; i < B; ++i) s += l.oneByOne[(size_t) (o * B + i)] * y[(size_t) i];
                    z[(size_t) o] += s;
                }
            }

            // head conv over the summed layer outputs
            a.headPos = (a.headPos + 1) % a.headHistLen;
            for (int c = 0; c < B; ++c) a.headHistory[(size_t) (a.headPos * B + c)] = head[(size_t) c];
            std::vector<double> h2 ((size_t) a.headSize, 0.0);
            for (int o = 0; o < a.headSize; ++o)
            {
                double s = a.headBias ? a.headConvBias[(size_t) o] : 0.0;
                for (int k = 0; k < a.headKernel; ++k)
                {
                    const int delay = a.headDilation * (a.headKernel - 1 - k);
                    const int idx = ((a.headPos - delay) % a.headHistLen + a.headHistLen) % a.headHistLen;
                    const double* h = &a.headHistory[(size_t) (idx * B)];
                    const double* wrow = &a.headConv[(size_t) (o * B * a.headKernel)];
                    for (int i = 0; i < B; ++i) s += wrow[i * a.headKernel + k] * h[i];
                }
                h2[(size_t) o] = s;
            }
            head = h2;
            in = z;
        }
        return (float) (head[0] * headScale);
    }

private:
    struct Act { enum Kind { Tanh, Leaky } kind = Tanh; double slope = 0.0; };
    struct Layer
    {
        int dilation = 1, kernel = 3, histLen = 1, pos = 0;
        Act act;
        std::vector<double> conv, convBias, mixin, oneByOne, oneByOneBias, history;
    };
    struct Array
    {
        int inputSize = 1, conditionSize = 1, headSize = 1, channels = 1, bottleneck = 1;
        int headKernel = 1, headDilation = 1, headHistLen = 1, headPos = 0;
        bool headBias = false;
        std::vector<int> dilations, kernels;
        std::vector<Act> acts;
        std::vector<double> rechannel, headConv, headConvBias, headHistory;
        std::vector<Layer> layers;
    };

    std::vector<Array> arrays;
    double headScale = 1.0, sampleRate = 48000.0;
    std::string name;
};
} // namespace namcmp
