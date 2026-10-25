#pragma once
#include <algorithm>
#include <cmath>

namespace vamp
{
// The 5150 high-voltage supply chain (factory schematic sheet 2, "EVH 120 preamp bd."):
//
//   power transformer HV winding -> bridge CR8-CR11 (1N4007) -> PLATE (C34 + C38 330u in series, bleeders R62 + R64)
//   -> R210 400 -> SCREEN (C35 + C39 100u in series, bleeders R63 + R65) -> R59 4k -> V3+ (C36 47u: phase inverter)
//   -> R60 22k -> V2+ (C37 22u: V2B V5B V5A V3A V3B) -> R61 10k -> V1+ (C33 22u: V1A V1B V2A)
//
// The bridge only conducts near the peaks of the mains: the reservoir is topped up in pulses, B+ carries a 120 Hz
// ripple (~6 V pp at idle, ~10 V playing hard) and sags with the current through the winding resistance and the
// conduction angle. Every rail sags with the current of the stages it feeds and recovers with the RC time constants
// (V3 188 ms, V2 484 ms, V1 220 ms, SCREEN 20 ms). step() integrates one sample with the loads' currents (explicit
// Euler: the fastest time constant, the winding resistance into the reservoir, is ~6.8 ms, >= ~1300 samples).
struct Supply5150
{
    static constexpr double kCplate = 165e-6, kR210 = 400.0, kCscreen = 50e-6;
    static constexpr double kR59 = 4e3, kC36 = 47e-6, kR60 = 22e3, kC37 = 22e-6, kR61 = 10e3, kC33 = 22e-6;
    static constexpr double kRbleed = 440e3; // R62 + R64 on PLATE, R63 + R65 on SCREEN (220k each)
    // Power transformer (not on the schematic): HV secondary 350 VAC at 600 mA (replacement for the 5150 / 6505,
    // Tube Amp Doctor). Winding resistance (primary referred to the secondary included) for ~7 % regulation, typical
    // for a ~280 VA transformer: 374.5 V unloaded, 41 ohm. Gives 488 V idle B+ (measured on 5150 / 6505s: 465-500 V).
    // 60 Hz mains (the US amp); two 1N4007 drops in the bridge path.
    static constexpr double kEmfRms = 374.5, kRwinding = 41.0, kMainsHz = 60.0, kDiodeDrops = 1.6;

    double plate = 487.7, screen = 479.6, v3 = 445.0, v2 = 317.5, v1 = 301.2;
    double phase = 0.0; // mains phase (cycles, 0..1)

    // charging current from the bridge at the present mains phase
    double charge() const
    {
        const double e = kEmfRms * std::sqrt (2.0) * std::abs (std::sin (2.0 * 3.14159265358979323846 * phase));
        return std::max (0.0, (e - kDiodeDrops - plate) / kRwinding);
    }

    // Operating point for constant load currents (A): the reservoir is run over the mains cycles until its ripple
    // repeats (from phase 0, so step() continues it seamlessly), the rest of the chain is the resistive DC solution.
    void setDC (double iPlate, double iScreen, double i3, double i2, double i1)
    {
        const double iR61 = i1, iR60 = i2 + iR61, iR59 = i3 + iR60;
        const double iScreenTotal = iScreen + iR59; // through R210, plus the screen bleeders below
        double iR210 = iScreenTotal + (plate - kR210 * iScreenTotal) / kRbleed;
        constexpr int kStepsPerCycle = 4000;
        const double h = 1.0 / (kMainsHz * kStepsPerCycle);
        phase = 0.0;
        for (int cycle = 0; cycle < 400; ++cycle)
        {
            const double start = plate;
            for (int k = 0; k < kStepsPerCycle; ++k)
            {
                plate += h / kCplate * (charge() - iPlate - iR210 - plate / kRbleed);
                phase += 1.0 / kStepsPerCycle;
            }
            phase = 0.0;
            iR210 = iScreenTotal + (plate - kR210 * iScreenTotal) / kRbleed;
            if (cycle > 20 && std::abs (plate - start) < 1e-6) break;
        }
        // the filter nodes at the average plate voltage (their ripple is filtered by R210 / C35 and beyond)
        double avg = 0.0, p = plate;
        for (int k = 0; k < kStepsPerCycle; ++k)
        {
            p += h / kCplate * (std::max (0.0, (kEmfRms * std::sqrt (2.0) * std::abs (std::sin (2.0 * 3.14159265358979323846 * k / kStepsPerCycle))
                                                - kDiodeDrops - p) / kRwinding) - iPlate - iR210 - p / kRbleed);
            avg += p;
        }
        avg /= kStepsPerCycle;
        screen = avg - kR210 * iR210;
        v3 = screen - kR59 * iR59;
        v2 = v3 - kR60 * iR60;
        v1 = v2 - kR61 * iR61;
    }

    // h: time step (s); currents (A) drawn from each node
    void step (double h, double iPlate, double iScreen, double i3, double i2, double i1)
    {
        const double iR210 = (plate - screen) / kR210, iR59 = (screen - v3) / kR59;
        const double iR60 = (v3 - v2) / kR60, iR61 = (v2 - v1) / kR61;
        const double iCharge = charge(); // the bridge conducts only to charge
        plate += h / kCplate * (iCharge - iPlate - iR210 - plate / kRbleed);
        screen += h / kCscreen * (iR210 - iScreen - iR59 - screen / kRbleed);
        v3 += h / kC36 * (iR59 - i3 - iR60);
        v2 += h / kC37 * (iR60 - i2 - iR61);
        v1 += h / kC33 * (iR61 - i1);
        phase += h * kMainsHz;
        if (phase >= 1.0) phase -= 1.0;
    }
};
} // namespace vamp
