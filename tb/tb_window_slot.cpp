// tb_window_slot.cpp
// =====================================================================
// Verilator testbench for window_slot.sv.
//
// Two test sections:
//
//   TEST 1 (single arrivals, spaced out)
//   Mirrors the directed test from tb_window_slot.cpp, but accounts for
//   the LATENCY-cycle delay between arrival_valid and cmp_valid:
//     1. Load P0 = (1000, 2000), idx=0
//     2. Drive arrival P1 = (1100, 2050), idx=1; wait LATENCY cycles;
//        verify cmp_dist=12500, cmp_tag=1, then verify best_idx/dist
//        update one cycle later
//     3. Drive P2 (far) and P3 (closer) likewise; verify best updates
//        as expected.
//
//   TEST 2 (back-to-back arrivals)
//   Drive P1, P2, P3 in consecutive cycles. Verify three cmp results
//   emerge in three consecutive cycles starting at LATENCY cycles
//   after the first drive. Final best should still settle to the same
//   value as in Test 1.
//
// Both tests use D=2 so LATENCY = $clog2(2) + 2 = 3 cycles.
// =====================================================================

#include "Vwindow_slot.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <iostream>
#include <string>

// -------- Pipeline depth must match the DUT param math --------
static constexpr int D       = 2;
static constexpr int LATENCY = 3;   // $clog2(2) + 2

// -------- Simulation infrastructure --------
static vluint64_t main_time = 0;
double sc_time_stamp() { return (double)main_time; }

static int n_checks   = 0;
static int n_failures = 0;

static void check(const std::string& label, uint64_t got, uint64_t expected) {
    n_checks++;
    if (got == expected) {
        std::cout << "  ok  : " << label << " = " << got << "\n";
    } else {
        std::cerr << "  FAIL: " << label
                  << "   got "      << got
                  << "   expected " << expected
                  << "   diff " << ((int64_t)got - (int64_t)expected) << "\n";
        n_failures++;
    }
}

static uint32_t pack2(int16_t d0, int16_t d1) {
    return (uint32_t)(uint16_t)d1 << 16 | (uint32_t)(uint16_t)d0;
}

static void tick(Vwindow_slot* dut, VerilatedVcdC* tfp) {
    dut->clk = 0; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
    dut->clk = 1; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
}

static void zero_inputs(Vwindow_slot* dut) {
    dut->load_valid    = 0;
    dut->load_idx      = 0;
    dut->load_data     = 0;
    dut->arrival_valid = 0;
    dut->arrival_idx   = 0;
    dut->arrival_data  = 0;
}

// Helpers that capture the common drive patterns
static void load_resident(Vwindow_slot* dut, VerilatedVcdC* tfp,
                          int16_t x, int16_t y, uint16_t idx) {
    dut->load_valid = 1;
    dut->load_idx   = idx;
    dut->load_data  = pack2(x, y);
    tick(dut, tfp);
    dut->load_valid = 0;
    dut->load_data  = 0;
    tick(dut, tfp);    // settle
}

static void reset(Vwindow_slot* dut, VerilatedVcdC* tfp) {
    zero_inputs(dut);
    dut->rst_n = 0;
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    dut->rst_n = 1;
    tick(dut, tfp);
}

// =====================================================================
// TEST 1 -- spaced arrivals
// =====================================================================
static void test1_spaced(Vwindow_slot* dut, VerilatedVcdC* tfp) {
    std::cout << "\n========== TEST 1: spaced arrivals ==========\n";

    reset(dut, tfp);
    std::cout << "\n[1] After reset:\n";
    check("resident_valid", dut->resident_valid, 0);

    load_resident(dut, tfp, 1000, 2000, 0);
    std::cout << "\n[2] After loading P0 = (1000, 2000), idx=0:\n";
    check("resident_valid", dut->resident_valid, 1);
    check("resident_idx",   dut->resident_idx,   0);

    // ---- Arrival P1: (1100, 2050), idx=1. dist^2 = 12500. Should update best. ----
    std::cout << "\n[3] Drive arrival P1 = (1100, 2050), idx=1. Expect dist^2 = 12500.\n";
    dut->arrival_valid = 1;
    dut->arrival_idx   = 1;
    dut->arrival_data  = pack2(1100, 2050);
    tick(dut, tfp);                      // tick 1: arrival latched into stage 1
    dut->arrival_valid = 0;
    dut->arrival_data  = 0;

    // Wait LATENCY-1 more ticks for cmp to emerge.
    // After tick LATENCY, valid_pipe[LATENCY-1]=1, so cmp_valid=1.
    for (int i = 0; i < LATENCY - 1; i++) tick(dut, tfp);

    check("cmp_valid (P1 emerges)", dut->cmp_valid, 1);
    check("cmp_tag",                dut->cmp_tag,   1);
    check("cmp_dist",               dut->cmp_dist,  12500);

    // One more tick: best update fires on this cmp_valid
    tick(dut, tfp);
    check("best_idx after P1",  dut->best_idx,  1);
    check("best_dist after P1", dut->best_dist, 12500);
    check("cmp_valid drained",  dut->cmp_valid, 0);

    // ---- Arrival P2: (5000, 4000), idx=2. dist^2 = 20,000,000. Should NOT update. ----
    std::cout << "\n[4] Drive arrival P2 = (5000, 4000), idx=2. Expect dist^2 = 20M, NO update.\n";
    dut->arrival_valid = 1;
    dut->arrival_idx   = 2;
    dut->arrival_data  = pack2(5000, 4000);
    tick(dut, tfp);
    dut->arrival_valid = 0;
    dut->arrival_data  = 0;
    for (int i = 0; i < LATENCY - 1; i++) tick(dut, tfp);

    check("cmp_valid (P2 emerges)", dut->cmp_valid, 1);
    check("cmp_tag",                dut->cmp_tag,   2);
    check("cmp_dist",               dut->cmp_dist,  20000000);

    tick(dut, tfp);
    check("best_idx unchanged",  dut->best_idx,  1);
    check("best_dist unchanged", dut->best_dist, 12500);

    // ---- Arrival P3: (1050, 2025), idx=3. dist^2 = 3125. Should update. ----
    std::cout << "\n[5] Drive arrival P3 = (1050, 2025), idx=3. Expect dist^2 = 3125, update.\n";
    dut->arrival_valid = 1;
    dut->arrival_idx   = 3;
    dut->arrival_data  = pack2(1050, 2025);
    tick(dut, tfp);
    dut->arrival_valid = 0;
    dut->arrival_data  = 0;
    for (int i = 0; i < LATENCY - 1; i++) tick(dut, tfp);

    check("cmp_valid (P3 emerges)", dut->cmp_valid, 1);
    check("cmp_tag",                dut->cmp_tag,   3);
    check("cmp_dist",               dut->cmp_dist,  3125);

    tick(dut, tfp);
    check("best_idx after P3",  dut->best_idx,  3);
    check("best_dist after P3", dut->best_dist, 3125);
}

// =====================================================================
// TEST 2 -- back-to-back arrivals
// =====================================================================
static void test2_streaming(Vwindow_slot* dut, VerilatedVcdC* tfp) {
    std::cout << "\n========== TEST 2: back-to-back arrivals ==========\n";

    reset(dut, tfp);
    load_resident(dut, tfp, 1000, 2000, 0);
    std::cout << "Loaded P0 = (1000, 2000), idx=0\n";

    // Drive three arrivals in three consecutive cycles.
    struct Arr { int16_t x, y; uint16_t idx; uint64_t expected; };
    Arr arrivals[3] = {
        {1100, 2050, 1, 12500},
        {5000, 4000, 2, 20000000},
        {1050, 2025, 3, 3125},
    };

    std::cout << "\nDriving 3 arrivals back-to-back...\n";
    for (int i = 0; i < 3; i++) {
        dut->arrival_valid = 1;
        dut->arrival_idx   = arrivals[i].idx;
        dut->arrival_data  = pack2(arrivals[i].x, arrivals[i].y);
        tick(dut, tfp);
    }
    dut->arrival_valid = 0;
    dut->arrival_data  = 0;

    // We drove 3 arrivals over 3 ticks. With LATENCY=3, the first
    // arrival's cmp emerges right after the 3rd tick -- i.e., cmp_valid
    // is high RIGHT NOW. The other two follow on the next two ticks.
    std::cout << "\n[after 3 drive ticks] First arrival should be emerging:\n";
    check("cmp_valid (A1)", dut->cmp_valid, 1);
    check("cmp_tag (A1)",   dut->cmp_tag,   arrivals[0].idx);
    check("cmp_dist (A1)",  dut->cmp_dist,  arrivals[0].expected);

    tick(dut, tfp);
    std::cout << "\n[tick +1] Second arrival should be emerging:\n";
    check("cmp_valid (A2)", dut->cmp_valid, 1);
    check("cmp_tag (A2)",   dut->cmp_tag,   arrivals[1].idx);
    check("cmp_dist (A2)",  dut->cmp_dist,  arrivals[1].expected);
    // Best should have been updated by A1's cmp (which emerged last
    // tick). Verify.
    check("best_idx (after A1 update)",  dut->best_idx,  arrivals[0].idx);
    check("best_dist (after A1 update)", dut->best_dist, arrivals[0].expected);

    tick(dut, tfp);
    std::cout << "\n[tick +2] Third arrival should be emerging:\n";
    check("cmp_valid (A3)", dut->cmp_valid, 1);
    check("cmp_tag (A3)",   dut->cmp_tag,   arrivals[2].idx);
    check("cmp_dist (A3)",  dut->cmp_dist,  arrivals[2].expected);
    // A2's cmp emerged last tick; it was larger so no best update.
    check("best_idx (A2 didn't update)",  dut->best_idx,  arrivals[0].idx);
    check("best_dist (A2 didn't update)", dut->best_dist, arrivals[0].expected);

    tick(dut, tfp);
    std::cout << "\n[tick +3] Pipeline drained; A3's update has taken effect:\n";
    check("cmp_valid drained",  dut->cmp_valid, 0);
    check("best_idx final",     dut->best_idx,  arrivals[2].idx);
    check("best_dist final",    dut->best_dist, arrivals[2].expected);
}

static void test3_loading(Vwindow_slot* dut, VerilatedVcdC* tfp) {
    std::cout << "\n========== TEST 3: load mechanism ==========\n";
 
    // Sentinel = all-ones across ACC_WIDTH=40 bits = 2^40 - 1.
    static constexpr uint64_t SENTINEL = (1ULL << 40) - 1;
 
    // Helper: drive a single arrival, wait for cmp to emerge and the
    // running-best register to settle. Centralises the LATENCY+1 timing
    // so the test body stays readable.
    auto drive_one_arrival = [&](int16_t x, int16_t y, uint16_t idx) {
        dut->arrival_valid = 1;
        dut->arrival_idx   = idx;
        dut->arrival_data  = pack2(x, y);
        tick(dut, tfp);                          // arrival latched into stage 1
        dut->arrival_valid = 0;
        dut->arrival_data  = 0;
        for (int i = 0; i < LATENCY - 1; i++)    // wait for cmp_valid
            tick(dut, tfp);
        tick(dut, tfp);                          // best update fires
    };
 
    // -----------------------------------------------------------------
    // 3a. Load-after-arrivals: best must reset to sentinel on reload
    // -----------------------------------------------------------------
    std::cout << "\n[3a] Load -> arrivals -> reload. Best must reset.\n";
 
    reset(dut, tfp);
    load_resident(dut, tfp, 1000, 2000, 0);
 
    // Drive an arrival that updates best to a non-sentinel value.
    drive_one_arrival(1100, 2050, 1);            // dist^2 = 12500
    check("3a: best_dist updated by first arrival", dut->best_dist, 12500);
    check("3a: best_idx updated by first arrival",  dut->best_idx,  1);
 
    // Now load a DIFFERENT resident. Best must reset.
    load_resident(dut, tfp, 5000, 5000, 99);
    check("3a: resident_idx after reload",   dut->resident_idx, 99);
    check("3a: resident_valid after reload", dut->resident_valid, 1);
    check("3a: best_dist reset to sentinel", dut->best_dist, SENTINEL);
    check("3a: best_idx cleared",            dut->best_idx,  0);
 
    // -----------------------------------------------------------------
    // 3b. Multi-epoch reuse: 3 consecutive (load, arrivals) cycles
    // -----------------------------------------------------------------
    std::cout << "\n[3b] Three epochs: load + arrivals, load + arrivals, ...\n";
 
    reset(dut, tfp);
 
    struct Epoch {
        int16_t  rx, ry;          uint16_t  ridx;     // resident
        int16_t  ax, ay;          uint16_t  aidx;     // single arrival
        uint64_t expected_dist;   uint16_t  expected_best_idx;
    };
    // Each epoch loads a distinct resident, drives one close-by arrival,
    // and verifies the resulting best is specific to that epoch.
    Epoch epochs[3] = {
        // R=(100,100), A=(110,100)         dist = 10^2 = 100
        {100, 100, 10, 110, 100, 11,  100, 11},
        // R=(1000,2000), A=(1100,2050)     dist = 100^2 + 50^2 = 12500
        {1000, 2000, 20, 1100, 2050, 21, 12500, 21},
        // R=(-500,300), A=(-490, 310)      dist = 10^2 + 10^2 = 200
        {-500, 300, 30, -490, 310, 31, 200, 31},
    };
    for (int e = 0; e < 3; e++) {
        const Epoch& ep = epochs[e];
        std::cout << "  -- epoch " << e
                  << "  load R=(" << ep.rx << "," << ep.ry << ") idx=" << ep.ridx
                  << "  arr A=(" << ep.ax << "," << ep.ay << ") idx=" << ep.aidx
                  << "  expected dist=" << ep.expected_dist << "\n";
        load_resident(dut, tfp, ep.rx, ep.ry, ep.ridx);
        check("3b: resident_idx", dut->resident_idx, ep.ridx);
        check("3b: best reset to sentinel on load", dut->best_dist, SENTINEL);
 
        drive_one_arrival(ep.ax, ep.ay, ep.aidx);
        check("3b: best_idx after arrival",  dut->best_idx,  ep.expected_best_idx);
        check("3b: best_dist after arrival", dut->best_dist, ep.expected_dist);
    }
 
    // -----------------------------------------------------------------
    // 3c. Load + arrival on the same cycle. The arrival MUST be dropped.
    // -----------------------------------------------------------------
    std::cout << "\n[3c] Load and arrival asserted in the same cycle.\n";
 
    reset(dut, tfp);
    load_resident(dut, tfp, 0, 0, 0);            // baseline resident
 
    // Assert load_valid and arrival_valid simultaneously. The slot should
    // load the new resident cleanly and drop the coincident arrival --
    // i.e., that arrival's distance must never reach best_dist.
    dut->load_valid    = 1;
    dut->load_idx      = 77;
    dut->load_data     = pack2(2000, 3000);
    dut->arrival_valid = 1;
    dut->arrival_idx   = 78;
    dut->arrival_data  = pack2(2001, 3001);      // would have dist^2 = 2 if it landed
    tick(dut, tfp);
    dut->load_valid    = 0;
    dut->load_data     = 0;
    dut->arrival_valid = 0;
    dut->arrival_data  = 0;
    tick(dut, tfp);                              // settle
 
    check("3c: resident_idx (new)",      dut->resident_idx, 77);
    check("3c: resident_valid (new)",    dut->resident_valid, 1);
    check("3c: best_dist still sentinel right after load", dut->best_dist, SENTINEL);
 
    // Now wait LATENCY+1 cycles. If the coincident arrival was NOT
    // properly suppressed, its result would emerge during this window
    // and contaminate best_dist (dropping it to 2). If suppression
    // works, best_dist stays at sentinel.
    for (int i = 0; i < LATENCY + 1; i++) tick(dut, tfp);
 
    check("3c: best_dist still sentinel after drain",  dut->best_dist, SENTINEL);
    check("3c: best_idx still cleared after drain",    dut->best_idx,  0);
 
    // And verify the slot is healthy afterward -- drive a real arrival
    // and confirm it updates best correctly. This catches the failure
    // mode where suppression also broke the slot's normal arrival path.
    drive_one_arrival(2010, 3000, 79);            // dist^2 = 10^2 = 100
    check("3c: post-coincidence, normal arrival updates best",
          dut->best_dist, 100);
    check("3c: post-coincidence, best_idx correct",
          dut->best_idx, 79);
}


// =====================================================================
// Main
// =====================================================================
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    auto* dut = new Vwindow_slot;
    auto* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves_window_slot.vcd");

    std::cout << "=== tb_window_slot ===  LATENCY = " << LATENCY << " cycles\n";

    test1_spaced  (dut, tfp);
    test2_streaming(dut, tfp);
    test3_loading  (dut, tfp);

    // Drain
    for (int i = 0; i < 4; i++) tick(dut, tfp);

    tfp->close();
    delete tfp;
    delete dut;

    std::cout << "\n=== Summary ===\n";
    std::cout << "  checks : " << n_checks << "\n";
    std::cout << "  passed : " << (n_checks - n_failures) << "\n";
    std::cout << "  failed : " << n_failures << "\n";
    std::cout << "  waveform: waves_window_slot.vcd\n";

    if (n_failures == 0) {
        std::cout << "\nPASS\n";
        return 0;
    } else {
        std::cerr << "\nFAIL: see above; open waves_window_slot.vcd in GTKWave.\n";
        return 1;
    }
}