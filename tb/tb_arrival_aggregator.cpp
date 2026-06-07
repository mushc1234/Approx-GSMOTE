// tb_arrival_aggregator.cpp
// =====================================================================
// Testbench for arrival_aggregator.sv -- per-level pipelined version.
//
// The aggregator is now pipelined per tree level: latency = $clog2(W)
// cycles, with the valid bit shift-registered alongside the data.
//
// The engine's slot tag-matches against the aggregator output every
// cycle, not just at LOAD, so we explicitly verify streaming behaviour
// (back-to-back different inputs) and bubble handling (invalid cycles
// in the middle of a valid stream). Those are the two failure modes
// the old (combinational-tree, 1-cycle-reg) design could not exhibit
// but the new one can.
//
// ASSUMPTION: the trailing single output register from the old design
// was dropped (the final tree level is now itself a register), so
// total latency = TREE_DEPTH. If you kept the trailing register, bump
// LATENCY below by 1.
//
// Directed tests:
//   1. Basic min selection across W inputs, all valid.
//   2. Min selection with some invalid slots -- invalids must lose.
//   3. All-invalid case -- output valid must stay low.
//   4. Tag tracking through tie-breaking (lower index wins).
//   5. Exact latency confirmation (was 1, now $clog2(W)).
//   6. Streaming: 4 back-to-back distinct inputs, outputs in order.
//   7. Bubble: invalid cycle in the middle of a valid stream propagates
//      to the right output cycle (valid shift register alignment).
// =====================================================================

#include "Varrival_aggregator.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <iostream>
#include <string>

static constexpr int W          = 4;
static constexpr int TREE_DEPTH = 2;          // = log2(W); update if W changes.
static constexpr int LATENCY    = TREE_DEPTH + 1; // input -> output, in cycles.

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

static void tick(Varrival_aggregator* dut, VerilatedVcdC* tfp) {
    dut->clk = 0; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
    dut->clk = 1; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
}

// Tick LATENCY times; after this, the output reflects the inputs that
// were on the bus when this was called (assuming inputs held stable).
static void advance_pipeline(Varrival_aggregator* dut, VerilatedVcdC* tfp) {
    for (int i = 0; i < LATENCY; i++) tick(dut, tfp);
}

static void zero_inputs(Varrival_aggregator* dut) {
    for (int i = 0; i < W; i++) {
        dut->slot_cmp_valid[i] = 0;
        dut->slot_cmp_dist [i] = 0;
        dut->slot_cmp_tag  [i] = 0;
    }
}

static void reset(Varrival_aggregator* dut, VerilatedVcdC* tfp) {
    zero_inputs(dut);
    dut->rst_n = 0;
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    dut->rst_n = 1;
    tick(dut, tfp);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    auto* dut = new Varrival_aggregator;
    auto* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves_arrival_aggregator.vcd");

    std::cout << "=== tb_arrival_aggregator ===  W=" << W
              << "  LATENCY=" << LATENCY << "\n";

    // -----------------------------------------------------------------
    // Test 1: basic min selection across all valid slots
    // -----------------------------------------------------------------
    std::cout << "\n[1] All slots valid; slot 2 has the smallest dist.\n";
    reset(dut, tfp);
    dut->slot_cmp_valid[0] = 1; dut->slot_cmp_dist[0] = 5000; dut->slot_cmp_tag[0] = 0xAA00;
    dut->slot_cmp_valid[1] = 1; dut->slot_cmp_dist[1] = 3000; dut->slot_cmp_tag[1] = 0xAA01;
    dut->slot_cmp_valid[2] = 1; dut->slot_cmp_dist[2] = 1000; dut->slot_cmp_tag[2] = 0xAA02;
    dut->slot_cmp_valid[3] = 1; dut->slot_cmp_dist[3] = 4000; dut->slot_cmp_tag[3] = 0xAA03;
    dut->eval();
    advance_pipeline(dut, tfp);

    check("out_valid",    dut->out_valid,    1);
    check("out_min_dist", dut->out_min_dist, 1000);
    check("out_min_tag",  dut->out_min_tag,  0xAA02);

    // Drain the pipe to a known-empty state before next test
    zero_inputs(dut);
    advance_pipeline(dut, tfp);

    // -----------------------------------------------------------------
    // Test 2: some slots invalid; smallest-among-valid wins
    // -----------------------------------------------------------------
    std::cout << "\n[2] Slot 2 is invalid; smallest among valid is slot 1 (3000).\n";
    reset(dut, tfp);
    dut->slot_cmp_valid[0] = 1; dut->slot_cmp_dist[0] = 5000; dut->slot_cmp_tag[0] = 0xBB00;
    dut->slot_cmp_valid[1] = 1; dut->slot_cmp_dist[1] = 3000; dut->slot_cmp_tag[1] = 0xBB01;
    dut->slot_cmp_valid[2] = 0; dut->slot_cmp_dist[2] = 1000; dut->slot_cmp_tag[2] = 0xBB02;
    dut->slot_cmp_valid[3] = 1; dut->slot_cmp_dist[3] = 4000; dut->slot_cmp_tag[3] = 0xBB03;
    dut->eval();
    advance_pipeline(dut, tfp);

    check("out_valid",    dut->out_valid,    1);
    check("out_min_dist", dut->out_min_dist, 3000);
    check("out_min_tag",  dut->out_min_tag,  0xBB01);

    // -----------------------------------------------------------------
    // Test 3: all slots invalid; out_valid must stay low across the
    // whole pipeline (not just at the final cycle).
    // -----------------------------------------------------------------
    std::cout << "\n[3] All slots invalid; out_valid stays low.\n";
    reset(dut, tfp);
    for (int i = 0; i < W; i++) {
        dut->slot_cmp_valid[i] = 0;
        dut->slot_cmp_dist [i] = 1234 + i;
        dut->slot_cmp_tag  [i] = 0xCC00 + i;
    }
    dut->eval();
    for (int i = 0; i < LATENCY + 2; i++) {
        tick(dut, tfp);
        check("out_valid stays low while inputs all invalid", dut->out_valid, 0);
    }

    // -----------------------------------------------------------------
    // Test 4: tag is preserved through deep tie-breaking
    // -----------------------------------------------------------------
    std::cout << "\n[4] Ties resolved deterministically (lower-index wins).\n";
    reset(dut, tfp);
    dut->slot_cmp_valid[0] = 1; dut->slot_cmp_dist[0] = 2000; dut->slot_cmp_tag[0] = 0xDD00;
    dut->slot_cmp_valid[1] = 1; dut->slot_cmp_dist[1] = 2000; dut->slot_cmp_tag[1] = 0xDD01;
    dut->slot_cmp_valid[2] = 1; dut->slot_cmp_dist[2] = 2000; dut->slot_cmp_tag[2] = 0xDD02;
    dut->slot_cmp_valid[3] = 1; dut->slot_cmp_dist[3] = 2000; dut->slot_cmp_tag[3] = 0xDD03;
    dut->eval();
    advance_pipeline(dut, tfp);
    // With <= comparator, lower-index always wins ties => tag = 0xDD00
    check("tie-breaking: lower index wins", dut->out_min_tag, 0xDD00);


    // -----------------------------------------------------------------
    // Test 5: exact latency confirmation.
    // Drive one valid input, then check on each intermediate cycle that
    // out_valid is still low; only on cycle LATENCY does it assert.
    // This catches an off-by-one in the valid shift register depth.
    // -----------------------------------------------------------------
    std::cout << "\n[5] Latency = " << LATENCY
              << " cycles from input change to output.\n";
    reset(dut, tfp);
    dut->slot_cmp_valid[0] = 1; dut->slot_cmp_dist[0] = 9999; dut->slot_cmp_tag[0] = 0xEE00;
    for (int i = 1; i < W; i++) {
        dut->slot_cmp_valid[i] = 0;
        dut->slot_cmp_dist [i] = 0;
        dut->slot_cmp_tag  [i] = 0;
    }
    dut->eval();
    check("before any tick: out_valid still 0", dut->out_valid, 0);
    for (int i = 0; i < LATENCY - 1; i++) {
        tick(dut, tfp);
        check("intermediate cycle: out_valid still 0", dut->out_valid, 0);
    }
    tick(dut, tfp);
    check("at LATENCY: out_valid asserts",          dut->out_valid,    1);
    check("at LATENCY: out_min_dist = input dist",  dut->out_min_dist, 9999);
    check("at LATENCY: out_min_tag  = input tag",   dut->out_min_tag,  0xEE00);

    // -----------------------------------------------------------------
    // Test 6: streaming -- 4 back-to-back distinct input vectors.
    // Drives a new input every cycle, then verifies outputs emerge
    // in the same order, LATENCY cycles delayed.
    //
    // The engine architecture relies on this: slots tag-match the
    // aggregator's output every cycle, so the aggregator must produce
    // a coherent stream, not just one good answer after a drain.
    // -----------------------------------------------------------------
    std::cout << "\n[6] Streaming: 4 distinct inputs back-to-back.\n";
    reset(dut, tfp);

    struct Vec { uint32_t d[W]; uint32_t t[W]; uint32_t exp_dist; uint32_t exp_tag; };
    Vec vecs[4] = {
        // Slot mins:  3 different winners across the 4 cycles
        { {5000,3000,1000,4000}, {0xA0,0xA1,0xA2,0xA3}, 1000, 0xA2 },
        { {2000,8000,7000,6000}, {0xB0,0xB1,0xB2,0xB3}, 2000, 0xB0 },
        { {9000,9000,9000,500},  {0xC0,0xC1,0xC2,0xC3}, 500,  0xC3 },
        { {1500,1400,1300,1200}, {0xD0,0xD1,0xD2,0xD3}, 1200, 0xD3 },
    };

    // Drive vectors on cycles 0..3, then hold zeros while pipe drains.
    // After each rising edge, the output corresponds to the vector
    // driven LATENCY cycles earlier (once the pipeline has filled).
    for (int cyc = 0; cyc < 4 + LATENCY; cyc++) {
        if (cyc < 4) {
            for (int i = 0; i < W; i++) {
                dut->slot_cmp_valid[i] = 1;
                dut->slot_cmp_dist [i] = vecs[cyc].d[i];
                dut->slot_cmp_tag  [i] = vecs[cyc].t[i];
            }
        } else {
            zero_inputs(dut);
        }
        dut->eval();
        tick(dut, tfp);

        // Output for vecs[k] appears after k + LATENCY ticks.
        // i.e. at this point (just after tick #cyc), output is vecs[cyc - LATENCY].
        int produced = cyc - LATENCY + 1;  // index of vec just emerged
        if (produced >= 0 && produced < 4) {
            std::string lbl = "stream cyc " + std::to_string(cyc) +
                              " -> vec " + std::to_string(produced);
            check(lbl + " out_valid",    dut->out_valid,    1);
            check(lbl + " out_min_dist", dut->out_min_dist, vecs[produced].exp_dist);
            check(lbl + " out_min_tag",  dut->out_min_tag,  vecs[produced].exp_tag);
        }
    }

    // -----------------------------------------------------------------
    // Test 7: bubble in a valid stream.
    // Drive valid, invalid, valid on three successive cycles. The
    // output should show valid, invalid, valid in the same order
    // LATENCY cycles later. This is the test that catches valid-shift
    // misalignment -- the failure mode where data is right but the
    // valid bit pops on the wrong cycle.
    // -----------------------------------------------------------------
    std::cout << "\n[7] Bubble: valid/invalid/valid propagates aligned.\n";
    reset(dut, tfp);

    auto drive_all_valid = [&](uint32_t d, uint32_t t) {
        for (int i = 0; i < W; i++) {
            dut->slot_cmp_valid[i] = 1;
            dut->slot_cmp_dist [i] = d + i;   // ensures slot 0 wins
            dut->slot_cmp_tag  [i] = t + i;
        }
    };
    auto drive_all_invalid = [&]() {
        for (int i = 0; i < W; i++) {
            dut->slot_cmp_valid[i] = 0;
            dut->slot_cmp_dist [i] = 0xDEAD;  // poison; should be ignored
            dut->slot_cmp_tag  [i] = 0xBEEF;
        }
    };

    // Schedule: cyc0 = valid(A), cyc1 = invalid, cyc2 = valid(C), then drain.
    bool   exp_valid[3] = { true, false, true };
    uint32_t exp_dist[3] = { 7000, 0,    8000 };  // dist for cyc1 is don't-care
    uint32_t exp_tag [3] = { 0xF000, 0,  0xF100 };

    for (int cyc = 0; cyc < 3 + LATENCY; cyc++) {
        if      (cyc == 0) drive_all_valid(7000, 0xF000);
        else if (cyc == 1) drive_all_invalid();
        else if (cyc == 2) drive_all_valid(8000, 0xF100);
        else               zero_inputs(dut);
        dut->eval();
        tick(dut, tfp);

        int produced = cyc - LATENCY + 1;
        if (produced >= 0 && produced < 3) {
            std::string lbl = "bubble cyc " + std::to_string(cyc) +
                              " -> in " + std::to_string(produced);
            check(lbl + " out_valid", dut->out_valid, exp_valid[produced] ? 1 : 0);
            if (exp_valid[produced]) {
                check(lbl + " out_min_dist", dut->out_min_dist, exp_dist[produced]);
                check(lbl + " out_min_tag",  dut->out_min_tag,  exp_tag [produced]);
            }
        }
    }

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