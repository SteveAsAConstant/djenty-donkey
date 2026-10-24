#pragma once
#include <algorithm>

namespace vamp_baseline
{
// The 5150 high-voltage supply chain (factory schematic sheet 2, "EVH 120 preamp bd."):
//
//   bridge (V0 through Rs) -> PLATE (C34 + C38 330u in series) -> R210 400 -> SCREEN (C35 + C39 100u in series)
//   -> R59 4k -> V3+ (C36 47u: phase inverter) -> R60 22k -> V2+ (C37 22u: V2B V5B V5A V3A V3B)
//   -> R61 10k -> V1+ (C33 22u: V1A V1B V2A)
//
// Every rail sags with the current of the stages it feeds and recovers with the RC time constants (V3 188 ms,
// V2 484 ms, V1 220 ms, SCREEN 20 ms, PLATE ~10 ms). step() integrates one sample with the loads' currents
// (explicit Euler: every time constant is >= ~3500 samples at the internal rate).
struct Supply5150
{
    static constexpr double kCplate = 165e-6, kR210 = 400.0, kCscreen = 50e-6;
    static constexpr double kR59 = 4e3, kC36 = 47e-6, kR60 = 22e3, kC37 = 22e-6, kR61 = 10e3, kC33 = 22e-6;
    // assumptions (not on the schematic): unloaded B+ at the reservoir, rectifier + winding resistance
    static constexpr double kV0 = 500.0, kRs = 60.0;

    // idle: the DC solution with the model's tube currents
    double plate = 490.9, screen = 486.5, v3 = 449.9, v2 = 322.1, v1 = 305.7;

    // DC operating point for constant load currents (A): the rectifier conducts, no current in the capacitors
    void setDC (double iPlate, double iScreen, double i3, double i2, double i1)
    {
        const double iR61 = i1, iR60 = i2 + iR61, iR59 = i3 + iR60, iR210 = iScreen + iR59;
        plate = kV0 - kRs * (iPlate + iR210);
        screen = plate - kR210 * iR210;
        v3 = screen - kR59 * iR59;
        v2 = v3 - kR60 * iR60;
        v1 = v2 - kR61 * iR61;
    }

    // h: time step (s); currents (A) drawn from each node
    void step (double h, double iPlate, double iScreen, double i3, double i2, double i1)
    {
        const double iR210 = (plate - screen) / kR210, iR59 = (screen - v3) / kR59;
        const double iR60 = (v3 - v2) / kR60, iR61 = (v2 - v1) / kR61;
        const double iCharge = std::max (0.0, (kV0 - plate) / kRs); // rectifier conducts only to charge
        plate += h / kCplate * (iCharge - iPlate - iR210);
        screen += h / kCscreen * (iR210 - iScreen - iR59);
        v3 += h / kC36 * (iR59 - i3 - iR60);
        v2 += h / kC37 * (iR60 - i2 - iR61);
        v1 += h / kC33 * (iR61 - i1);
    }
};
} // namespace vamp_baseline
