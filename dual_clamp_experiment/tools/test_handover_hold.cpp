#include "../HandoverHold.h"
#include "../ClampCurveBuffer.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <limits>
#include <vector>
#include <iomanip>

using namespace handoverhold;
void check(bool ok, const char* text) { if (!ok) throw std::runtime_error(text); }
struct Rig {
    Tracker tracker;
    Input in{0, 0, 4, 1, 9, 3, 1, 4, true, true};
    Result tick(unsigned phase, double x, double fn = 1, double torque = 4) {
        in.phase = phase; in.position_mm = x; in.fn_N = fn; in.torque_Nmm = torque;
        auto result = tracker.update(in); ++in.index; in.time_us += 1000; return result;
    }
    Result acquire(bool stable = true) {
        Result r;
        for (int i = 0; i <= 600; ++i)
            r = tick(4, 9 - i * .01, stable ? 1 : double(i % 2), i * .1);
        return r;
    }
};
void tests() {
    Rig a;
    auto r = a.acquire();
    check(r.reference_valid && r.sample_count == 20 && r.source == Source::StableWindow, "stable reference");
    check(r.reference_start_index == 149 && !r.holding && r.fn_N == 1, "all 50 points inside 5-2mm");
    r = a.tick(5, 3, 9, 90);
    check(r.holding && r.fn_N == 1 && std::abs(r.torque_Nmm - 20.15) < 1e-9, "same 20 spatial points both channels");
    for (unsigned p : {5, 6, 7, 8}) check(a.tick(p, 23, 9, 90).holding, "handover coverage");
    ++a.in.cycle;
    check(a.tick(4, 23, 9, 90).holding, "next forward origin");
    check(a.tick(4, 21.001, 9, 90).holding, "hold before 2mm");
    r = a.tick(4, 21, 9, 90);
    check(!r.holding && r.status == Status::Resumed && r.fn_N == 9 && r.torque_Nmm == 90, "immediate recovery at 2mm");
    r = a.tick(5, 3, 9, 90);
    check(!r.valid && !r.reference_valid, "no reuse of previous cycle");

    Rig b; r = b.acquire(false);
    check(r.reference_valid && r.source == Source::ForcedAt2mm && r.reference_start_index == 400, "forced sampling");
    b.tick(5, 3); b.tick(6, 23); b.tick(7, 23); b.tick(8, 23);
    check(b.tick(9, 23).holding && !b.tick(9, 21).holding, "final forward recovery");

    Rig c; c.acquire(); c.tick(5, 3);
    r = c.tick(10, 23, 100, 100);
    check(!r.valid && !r.holding && r.status == Status::Ended, "terminal ends without recovery");
    c.acquire(); c.tick(5, 3); c.tracker.reset();
    check(!c.tick(6, 23).valid, "explicit reset discards hold");

    for (int kind = 0; kind < 4; ++kind) {
        Rig d; d.acquire(); d.tick(5, 3);
        if (kind == 0) d.in.time_us += 1000;
        if (kind == 1) ++d.in.index;
        if (kind == 2) d.in.valid = false;
        if (kind == 3) d.in.catheter = false;
        r = d.tick(6, 20);
        check(!r.valid && !r.holding, "gap/invalid/mode resets hold");
        d.in.valid = d.in.catheter = true;
        check(!d.tick(7, 23).holding, "no post-reset stale reference");
    }
    Rig e;
    for (int i = 0; i < 49; ++i) e.tick(4, 7.9);
    e.tick(4, 8.1);
    for (int i = 0; i < 49; ++i) r = e.tick(4, 7.9);
    check(r.source == Source::None, "stable window cannot cross position boundary");
    e.tick(4, 7.9);
    r = e.tick(4, 7.7);
    check(r.status == Status::SpatialUndersampled && !r.reference_valid, "no duplicate spatial samples");
    check(!e.tick(5, 3).valid, "undersampled reference unavailable");

    Rig f; f.tick(4, 5);
    for (int i = 0; i < 100; ++i) r = f.tick(4, 5);
    check(r.sample_count == 0, "stationary samples do not count as distance");
    r = f.tick(4, 4.95); check(r.sample_count == 1, "first spatial crossing");
    f.tick(4, 5.1); r = f.tick(4, 4.95);
    check(r.sample_count == 1, "position reversal does not resample crossing");
    r = f.tick(4, 4.9, std::numeric_limits<double>::quiet_NaN());
    check(!r.valid && r.status == Status::InvalidInput, "nonfinite force");

    clampmodel::CurveBuffer buffer;
    clampmodel::CurvePoint point; point.handover_fn = 1; point.handover_ft = 2;
    point.handover_valid = point.handover_holding = true;
    buffer.push(point);
    auto response = buffer.response(0, 0, 1, true, true, true, 0, 0);
    auto fields = response.substr(response.rfind('|') + 1);
    check(std::count(fields.begin(), fields.end(), ',') == 24, "curve protocol append-only 25 fields");
    check(fields.find(",1,2,1,0,1,0,0") != std::string::npos, "curve hold payload");
    std::ostringstream csv; write_csv(csv, Result{});
    check(csv.str().find(",,,0,0,0,none,inactive") == 0, "invalid values blank");
    std::cout << "Handover state, spatial sampling, reset and protocol tests passed\n";
}
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--replay") {
            Tracker tracker; Input in;
            std::cout << std::setprecision(17) << "sample_index";
            write_csv_header(std::cout); std::cout << '\n';
            while (std::cin >> in.time_us >> in.index >> in.phase >> in.cycle >> in.position_mm
                >> in.trigger_mm >> in.fn_N >> in.torque_Nmm >> in.valid >> in.catheter) {
                auto r = tracker.update(in);
                std::cout << in.index; write_csv(std::cout, r); std::cout << '\n';
            }
            check(std::cin.eof(), "invalid replay input");
        } else tests();
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
