// tb_arrival_aggregator.cpp
// =====================================================================
// Testbench for arrival_aggregator.sv. Directed tests covering:
//   1. Basic min selection across W inputs, all valid.
//   2. Min selection with some invalid slots -- invalids must lose.
//   3. All-invalid case -- output valid must stay low.
//   4. Tag tracking -- the output tag matches the winning slot.
//   5. 1-cycle latency confirmation.
//
// Uses W=4 for easy hand-construction of expected results. The DUT
// is parametric so the same tests apply to any power-of-2 W.
// =====================================================================

#include "Varrival_aggregator.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <iostream>
#include <string>

static constexpr int W = 4;

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

    std::cout << "=== tb_arrival_aggregator ===  W=" << W << "\n";

    // -----------------------------------------------------------------
    // Test 1: basic min selection across all valid slots
    // -----------------------------------------------------------------
    std::cout << "\n[1] All slots valid; slot 2 has the smallest dist.\n";
    reset(dut, tfp);
    dut->slot_cmp_valid[0] = 1; dut->slot_cmp_dist[0] = 5000; dut->slot_cmp_tag[0] = 0xAA00;
    dut->slot_cmp_valid[1] = 1; dut->slot_cmp_dist[1] = 3000; dut->slot_cmp_tag[1] = 0xAA01;
    dut->slot_cmp_valid[2] = 1; dut->slot_cmp_dist[2] = 1000; dut->slot_cmp_tag[2] = 0xAA02;
    dut->slot_cmp_valid[3] = 1; dut->slot_cmp_dist[3] = 4000; dut->slot_cmp_tag[3] = 0xAA03;
    dut->eval();          // settle combinational tree
    tick(dut, tfp);       // register output

    check("out_valid",    dut->out_valid,    1);
    check("out_min_dist", dut->out_min_dist, 1000);
    check("out_min_tag",  dut->out_min_tag,  0xAA02);

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
    tick(dut, tfp);

    check("out_valid",    dut->out_valid,    1);
    check("out_min_dist", dut->out_min_dist, 3000);
    check("out_min_tag",  dut->out_min_tag,  0xBB01);

    // -----------------------------------------------------------------
    // Test 3: all slots invalid; out_valid must stay low
    // -----------------------------------------------------------------
    std::cout << "\n[3] All slots invalid; out_valid stays low.\n";
    reset(dut, tfp);
    for (int i = 0; i < W; i++) {
        dut->slot_cmp_valid[i] = 0;
        dut->slot_cmp_dist [i] = 1234 + i;
        dut->slot_cmp_tag  [i] = 0xCC00 + i;
    }
    dut->eval();
    tick(dut, tfp);
    check("out_valid stays low when all inputs invalid", dut->out_valid, 0);

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
    tick(dut, tfp);
    // With <= comparator, lower-index always wins ties => tag = 0xDD00
    check("tie-breaking: lower index wins", dut->out_min_tag, 0xDD00);

    // -----------------------------------------------------------------
    // Test 5: latency confirmation - drive a known input, count cycles to output
    // -----------------------------------------------------------------
    std::cout << "\n[5] Latency = 1 cycle from input change to output.\n";
    reset(dut, tfp);
    // Drive inputs
    dut->slot_cmp_valid[0] = 1; dut->slot_cmp_dist[0] = 9999; dut->slot_cmp_tag[0] = 0xEE00;
    for (int i = 1; i < W; i++) {
        dut->slot_cmp_valid[i] = 0;
        dut->slot_cmp_dist [i] = 0;
        dut->slot_cmp_tag  [i] = 0;
    }
    dut->eval();
    // Before the clock edge, output reflects previous cycle (still low after reset)
    check("before tick: out_valid still 0", dut->out_valid, 0);
    tick(dut, tfp);
    check("after 1 tick: out_valid asserts", dut->out_valid, 1);
    check("after 1 tick: out_min_dist = input dist", dut->out_min_dist, 9999);

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
