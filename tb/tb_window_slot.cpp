// tb_window_slot.cpp
// =====================================================================
// Testbench for window_slot.sv (continuous-aggregator-tag-match version).
//
// Tests encode the INTENDED behaviour, not what the current RTL happens
// to do. Concretely:
//
//   1. The cmp path updates best ONLY when
//        cmp_valid && cmp_tag[upper] == resident_idx.
//      The upper-half check rejects in-flight compares from a previous
//      resident that emerge after a load.
//
//   2. The agg path updates best ONLY when
//        agg_best_tag[LOWER] == resident_idx.
//      The lower half of the aggregator tag is the arrival_idx the agg
//      result was computed for. When that arrival_idx is now this
//      slot's resident, the upper half (the closest-resident-to-it) is
//      the neighbour we should adopt as best_idx.
//
//   3. On load, best resets to (MAX, 0) and the slot's data latches in.
//      Load takes priority over both other paths in the same cycle.
//
//   4. cmp and agg can both fire on the same cycle; the smaller of the
//      two gated distances wins, and best_idx reflects the winner's
//      side (cmp_tag[lower] for cmp; agg_best_tag[upper] for agg).
//
// Parameters: D=4, IN_WIDTH=16, ACC_WIDTH=40, IDX_WIDTH=16. The slot
// must be elaborated with these for the packed-data helpers below to
// line up.
// =====================================================================

#include "Vwindow_slot.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <iostream>
#include <string>

static constexpr int D         = 4;
static constexpr int IN_WIDTH  = 16;
static constexpr int IDX_WIDTH = 16;
static constexpr uint64_t MAX_DIST = ((uint64_t)1 << 40) - 1;  // ACC_WIDTH=40

static vluint64_t main_time = 0;
double sc_time_stamp() { return (double)main_time; }

static int n_checks = 0, n_failures = 0;
static void check(const std::string& label, uint64_t got, uint64_t expected) {
    n_checks++;
    if (got == expected) {
        std::cout << "  ok  : " << label << " = " << got << "\n";
    } else {
        std::cerr << "  FAIL: " << label
                  << "   got " << got
                  << "   expected " << expected << "\n";
        n_failures++;
    }
}

static void tick(Vwindow_slot* dut, VerilatedVcdC* tfp) {
    dut->clk = 0; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
    dut->clk = 1; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
}

// Pack D=4 int16 values into a packed 64-bit signal.
// Element 0 occupies bits [15:0], element 1 [31:16], etc.
static uint64_t pack4(int16_t a, int16_t b, int16_t c, int16_t d) {
    return ((uint64_t)(uint16_t)a)        |
           ((uint64_t)(uint16_t)b) << 16  |
           ((uint64_t)(uint16_t)c) << 32  |
           ((uint64_t)(uint16_t)d) << 48;
}

// Squared euclidean distance for D=4 vectors.
static uint64_t sqdist(int16_t a0, int16_t a1, int16_t a2, int16_t a3,
                       int16_t b0, int16_t b1, int16_t b2, int16_t b3) {
    int64_t d0 = (int64_t)a0 - b0;
    int64_t d1 = (int64_t)a1 - b1;
    int64_t d2 = (int64_t)a2 - b2;
    int64_t d3 = (int64_t)a3 - b3;
    return (uint64_t)(d0*d0 + d1*d1 + d2*d2 + d3*d3);
}

// Drive agg into a quiescent (non-matching, MAX-dist) state. Any
// resident_idx in [0..0xFFFE] won't match the all-ones tag halves.
static void quiesce_agg(Vwindow_slot* dut) {
    dut->agg_best_tag  = 0xFFFFFFFFULL;
    dut->agg_best_dist = MAX_DIST;
}

static void zero_inputs(Vwindow_slot* dut) {
    dut->load_valid    = 0;
    dut->load_idx      = 0;
    dut->load_data     = 0;
    dut->arrival_valid = 0;
    dut->arrival_idx   = 0;
    dut->arrival_data  = 0;
    quiesce_agg(dut);
}

static void reset(Vwindow_slot* dut, VerilatedVcdC* tfp) {
    zero_inputs(dut);
    dut->rst_n = 0;
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    dut->rst_n = 1;
    tick(dut, tfp);
}

// One-cycle load pulse. After the tick: resident_valid=1, best=MAX.
static void do_load(Vwindow_slot* dut, VerilatedVcdC* tfp,
                    uint16_t idx, uint64_t data) {
    dut->load_valid = 1;
    dut->load_idx   = idx;
    dut->load_data  = data;
    tick(dut, tfp);
    dut->load_valid = 0;
    dut->load_data  = 0;
    dut->load_idx   = 0;
}

// One-cycle arrival broadcast. After the tick: dist_unit has captured
// the input (assuming resident_valid=1 and load_valid=0 at this cycle).
static void do_arrival(Vwindow_slot* dut, VerilatedVcdC* tfp,
                       uint16_t idx, uint64_t data) {
    dut->arrival_valid = 1;
    dut->arrival_idx   = idx;
    dut->arrival_data  = data;
    tick(dut, tfp);
    dut->arrival_valid = 0;
    dut->arrival_idx   = 0;
    dut->arrival_data  = 0;
}

// Tick until cmp_valid is observed=1 after the tick. Returns the
// number of ticks consumed, or -1 on timeout. Robust to dist_unit
// pipeline-depth changes (recompute latency if you change D).
static int wait_for_cmp(Vwindow_slot* dut, VerilatedVcdC* tfp,
                        int max_cycles = 32) {
    for (int i = 0; i < max_cycles; i++) {
        tick(dut, tfp);
        if (dut->cmp_valid) return i + 1;
    }
    return -1;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    auto* dut = new Vwindow_slot;
    auto* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves_window_slot.vcd");

    std::cout << "=== tb_window_slot ===  D=" << D
              << "  IDX_WIDTH=" << IDX_WIDTH << "\n";

    // Fixed test vectors. Distances picked to be easy to read.
    const uint16_t A = 0x0042, B = 0x00B0, X = 0x0001, Y = 0x0002;
    const uint16_t R_WIN = 0x0099;  // a winning-resident idx for agg tests

    // Resident A data:        [10, 20, 30, 40]
    // Arrival X data:         [11, 22, 31, 41]  -> dist(A,X) = 1+4+1+1 = 7
    // Arrival Y data:         [12, 24, 32, 42]  -> dist(A,Y) = 4+16+4+4 = 28
    const uint64_t A_DATA = pack4(10, 20, 30, 40);
    const uint64_t B_DATA = pack4(50, 50, 50, 50);
    const uint64_t X_DATA = pack4(11, 22, 31, 41);
    const uint64_t Y_DATA = pack4(12, 24, 32, 42);
    const uint64_t DIST_AX = sqdist(10,20,30,40, 11,22,31,41);  // 7
    const uint64_t DIST_AY = sqdist(10,20,30,40, 12,24,32,42);  // 28

    // -----------------------------------------------------------------
    // [1] Reset clears outputs.
    // -----------------------------------------------------------------
    std::cout << "\n[1] Reset state.\n";
    reset(dut, tfp);
    check("resident_valid", dut->resident_valid, 0);
    check("best_dist = MAX", dut->best_dist,     MAX_DIST);
    check("best_idx = 0",   dut->best_idx,       0);
    check("cmp_valid = 0",  dut->cmp_valid,      0);

    // -----------------------------------------------------------------
    // [2] Load latches resident; best resets to (MAX, 0).
    // -----------------------------------------------------------------
    std::cout << "\n[2] Load latches resident; best reset to MAX/0.\n";
    reset(dut, tfp);
    do_load(dut, tfp, A, A_DATA);
    check("resident_valid", dut->resident_valid, 1);
    check("resident_idx = A", dut->resident_idx, A);
    check("best_dist = MAX",  dut->best_dist,    MAX_DIST);
    check("best_idx = 0",     dut->best_idx,     0);

    // -----------------------------------------------------------------
    // [3] Single arrival -> cmp_valid pulses -> best updates from cmp.
    // Verifies: cmp_tag = {A, X}; cmp_dist = sqdist(A, X);
    //          best_dist = cmp_dist; best_idx = X (lower half of tag).
    // -----------------------------------------------------------------
    std::cout << "\n[3] Single compare via cmp path; best_idx = arrival_idx.\n";
    reset(dut, tfp);
    do_load(dut, tfp, A, A_DATA);
    do_arrival(dut, tfp, X, X_DATA);
    int waited = wait_for_cmp(dut, tfp, 16);
    check("cmp_valid eventually asserts", waited > 0 ? 1 : 0, 1);
    check("cmp_tag upper = A", (dut->cmp_tag >> IDX_WIDTH) & 0xFFFF, A);
    check("cmp_tag lower = X",  dut->cmp_tag & 0xFFFF,              X);
    check("cmp_dist = sqdist(A,X)", dut->cmp_dist, DIST_AX);
    tick(dut, tfp);  // one more cycle to register cmp into best
    check("best_dist = DIST_AX",  dut->best_dist, DIST_AX);
    check("best_idx  = X",        dut->best_idx,  X);

    // -----------------------------------------------------------------
    // [4] Second arrival, larger dist -> best unchanged.
    // -----------------------------------------------------------------
    std::cout << "\n[4] Second arrival farther; best unchanged.\n";
    // Continue from [3]: best = (DIST_AX, X)
    do_arrival(dut, tfp, Y, Y_DATA);
    waited = wait_for_cmp(dut, tfp, 16);
    check("cmp_valid asserts again", waited > 0 ? 1 : 0, 1);
    check("cmp_dist = sqdist(A,Y)",  dut->cmp_dist, DIST_AY);
    tick(dut, tfp);
    check("best_dist still DIST_AX", dut->best_dist, DIST_AX);
    check("best_idx  still X",       dut->best_idx,  X);

    // -----------------------------------------------------------------
    // [5] Second arrival, smaller dist -> best updates.
    // Reuse a fresh load to control the scenario: load A, drive Y
    // first (large), then X (small). After X completes, best should
    // be (DIST_AX, X).
    // -----------------------------------------------------------------
    std::cout << "\n[5] Smaller second compare overwrites best.\n";
    reset(dut, tfp);
    do_load(dut, tfp, A, A_DATA);
    do_arrival(dut, tfp, Y, Y_DATA);
    waited = wait_for_cmp(dut, tfp, 16);
    check("Y cmp fires", waited > 0 ? 1 : 0, 1);
    tick(dut, tfp);
    check("best_dist = DIST_AY first", dut->best_dist, DIST_AY);
    check("best_idx  = Y",             dut->best_idx,  Y);
    do_arrival(dut, tfp, X, X_DATA);
    waited = wait_for_cmp(dut, tfp, 16);
    check("X cmp fires", waited > 0 ? 1 : 0, 1);
    tick(dut, tfp);
    check("best_dist = DIST_AX now", dut->best_dist, DIST_AX);
    check("best_idx  = X",           dut->best_idx,  X);

    wait_for_cmp(dut, tfp, 16);  // Wait a bit to ensure no more cmp_valid pulses.

    // -----------------------------------------------------------------
    // [7] Aggregator snap-in with correctly-directed tag.
    //   agg_best_tag = {R_WIN, A}   -- lower half = A = resident_idx
    //   agg_best_dist = K
    // Expected: best_dist = K, best_idx = R_WIN (upper half).
    // arrival_valid stays 0 so cmp path can't fire.
    // -----------------------------------------------------------------
    std::cout << "\n[7] Agg snap with lower==resident -> best adopts agg.\n";
    reset(dut, tfp);
    do_load(dut, tfp, A, A_DATA);
    const uint64_t K_AGG = 100;
    dut->agg_best_tag  = ((uint32_t)R_WIN << IDX_WIDTH) | A;  // lower=A
    dut->agg_best_dist = K_AGG;
    tick(dut, tfp);  // slot samples agg on this posedge
    check("best_dist = K_AGG",  dut->best_dist, K_AGG);
    check("best_idx  = R_WIN",  dut->best_idx,  R_WIN);

    // -----------------------------------------------------------------
    // [8] Aggregator with WRONG-direction tag (upper==resident_idx,
    // lower != resident_idx). Intended behaviour: reject. This is
    // the test that disambiguates the lower-half match from the
    // upper-half match -- both fail-modes look the same on tests
    // with all-non-matching tags, but only the lower-half match
    // correctly rejects this case.
    // -----------------------------------------------------------------
    std::cout << "\n[8] Agg with upper==resident, lower!=resident: must reject.\n";
    reset(dut, tfp);
    do_load(dut, tfp, A, A_DATA);
    // Upper half = A (would falsely match if direction is inverted).
    // Lower half = X (arrival_idx not equal to our resident A).
    dut->agg_best_tag  = ((uint32_t)A << IDX_WIDTH) | X;
    dut->agg_best_dist = 50;  // small; would corrupt best if accepted
    tick(dut, tfp);
    for (int i = 0; i < 3; i++) tick(dut, tfp);
    check("best_dist still MAX (wrong tag direction rejected)",
          dut->best_dist, MAX_DIST);
    check("best_idx  still 0", dut->best_idx, 0);

    // -----------------------------------------------------------------
    // [9] Concurrent cmp + agg on the same cycle: smaller wins.
    // Strategy: start a compare; on the cycle cmp_valid rises, also
    // drive a matching agg with a SMALLER dist. Both paths are
    // gated valid on the next posedge; the agg side wins because
    // its dist is smaller, and best_idx = R_WIN.
    // -----------------------------------------------------------------
    std::cout << "\n[9] Concurrent agg+cmp; agg smaller -> agg wins.\n";
    reset(dut, tfp);
    do_load(dut, tfp, A, A_DATA);
    do_arrival(dut, tfp, X, X_DATA);
    waited = wait_for_cmp(dut, tfp, 16);
    check("cmp_valid fires", waited > 0 ? 1 : 0, 1);
    // Now cmp_valid=1 in the current cycle; slot will sample on next tick.
    // Inject matching agg with smaller dist than DIST_AX (= 7).
    dut->agg_best_tag  = ((uint32_t)R_WIN << IDX_WIDTH) | A;
    dut->agg_best_dist = 3;  // < DIST_AX
    tick(dut, tfp);
    check("best_dist = 3 (agg's)",   dut->best_dist, 3);
    check("best_idx  = R_WIN (agg)", dut->best_idx,  R_WIN);

    // Same setup, but agg's dist is LARGER than cmp's: cmp should win.
    std::cout << "\n[9b] Concurrent agg+cmp; cmp smaller -> cmp wins.\n";
    reset(dut, tfp);
    do_load(dut, tfp, A, A_DATA);
    do_arrival(dut, tfp, X, X_DATA);
    waited = wait_for_cmp(dut, tfp, 16);
    check("cmp_valid fires", waited > 0 ? 1 : 0, 1);
    dut->agg_best_tag  = ((uint32_t)R_WIN << IDX_WIDTH) | A;
    dut->agg_best_dist = 1000;  // > DIST_AX (= 7)
    tick(dut, tfp);
    check("best_dist = DIST_AX (cmp's)", dut->best_dist, DIST_AX);
    check("best_idx  = X (cmp)",         dut->best_idx,  X);

    // -----------------------------------------------------------------
    // [10] Load resets best even when best was previously populated.
    // -----------------------------------------------------------------
    std::cout << "\n[10] New load clears prior best.\n";
    // Continue from [9b]: best is (DIST_AX, X), resident A.
    do_load(dut, tfp, B, B_DATA);
    check("resident_idx = B",        dut->resident_idx, B);
    check("best_dist reset to MAX",  dut->best_dist,    MAX_DIST);
    check("best_idx reset to 0",     dut->best_idx,     0);

    for (int i = 0; i < 4; i++) tick(dut, tfp);
    tfp->close();
    delete tfp;
    delete dut;

    std::cout << "\n=== Summary ===\n";
    std::cout << "  checks : " << n_checks << "\n";
    std::cout << "  passed : " << (n_checks - n_failures) << "\n";
    std::cout << "  failed : " << n_failures << "\n";
    if (n_failures == 0) { std::cout << "\nPASS\n"; return 0; }
    else { std::cerr << "\nFAIL\n"; return 1; }
}