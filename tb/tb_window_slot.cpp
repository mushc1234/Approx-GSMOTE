// tb_window_slot.cpp
// =====================================================================
// Verilator testbench for window_slot.sv. Directed test exercising
// resident load, distance computation, and running-best update.
//
// Test sequence (4 points in D=2):
//   reset, then:
//   1. Load resident P0 = (1000, 2000), idx=0
//   2. Arrival P1 = (1100, 2050), idx=1  -> dist^2 =    12,500.  UPDATE best -> idx=1
//   3. Arrival P2 = (5000, 4000), idx=2  -> dist^2 = 20,000,000.  NO update (still idx=1)
//   4. Arrival P3 = (1050, 2025), idx=3  -> dist^2 =     3,125.  UPDATE best -> idx=3
//
// Final expected state: resident_idx=0, best_idx=3, best_dist=3125.
//
// On first build with the stub DUT, every check FAILS. Implement
// window_slot.sv piece by piece and re-run -- the failures localise
// exactly which feature is missing.
// =====================================================================

#include "Vwindow_slot.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <iostream>
#include <string>

// --------------------------------------------------------------------
// Simulation infrastructure
// --------------------------------------------------------------------
static vluint64_t main_time = 0;
double sc_time_stamp() { return (double)main_time; }

static int n_checks = 0;
static int n_failures = 0;

static void check(const std::string& label, uint64_t got, uint64_t expected) {
    n_checks++;
    if (got == expected) {
        std::cout << "  ok  : " << label
                  << " = " << got << "\n";
    } else {
        std::cerr << "  FAIL: " << label
                  << "   got " << got
                  << "   expected " << expected
                  << "   (diff " << ((int64_t)got - (int64_t)expected) << ")\n";
        n_failures++;
    }
}

// Pack two signed 16-bit dimensions into a 32-bit unsigned vector
// matching the SV port layout: dim 0 in [15:0], dim 1 in [31:16].
static uint32_t pack2(int16_t d0, int16_t d1) {
    return (uint32_t)(uint16_t)d1 << 16 | (uint32_t)(uint16_t)d0;
}

static void tick(Vwindow_slot* dut, VerilatedVcdC* tfp) {
    dut->clk = 0;
    dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;

    dut->clk = 1;
    dut->eval();
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

// --------------------------------------------------------------------
// Main
// --------------------------------------------------------------------
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    Vwindow_slot* dut = new Vwindow_slot;
    VerilatedVcdC* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves.vcd");

    std::cout << "=== tb_window_slot ===\n";

    // ---------- Reset ----------
    zero_inputs(dut);
    dut->rst_n = 0;
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    dut->rst_n = 1;
    tick(dut, tfp);

    std::cout << "\n[1] After reset:\n";
    check("resident_valid (should be 0 after reset)", dut->resident_valid, 0);

    // ---------- Load resident P0 = (1000, 2000), idx=0 ----------
    dut->load_valid = 1;
    dut->load_idx   = 0;
    dut->load_data  = pack2(1000, 2000);
    tick(dut, tfp);
    dut->load_valid = 0;
    dut->load_data  = 0;
    tick(dut, tfp);  // settle

    std::cout << "\n[2] After loading resident P0=(1000,2000), idx=0:\n";
    check("resident_valid", dut->resident_valid, 1);
    check("resident_idx",   dut->resident_idx,   0);

    // ---------- Arrival P1 = (1100, 2050), idx=1.  dist^2 = 100^2 + 50^2 = 12500 ----------
    dut->arrival_valid = 1;
    dut->arrival_idx   = 1;
    dut->arrival_data  = pack2(1100, 2050);
    dut->eval();  // combinational settle so cmp_dist reflects current inputs

    std::cout << "\n[3] Arrival P1=(1100,2050), idx=1.  Expected dist^2 = 12500:\n";
    check("cmp_dist for P1", dut->cmp_dist, 12500);
    tick(dut, tfp);          // clock edge: best should update
    dut->arrival_valid = 0;
    dut->arrival_data  = 0;
    tick(dut, tfp);          // settle

    check("best_idx after P1 (should update to 1)",  dut->best_idx,  1);
    check("best_dist after P1 (should be 12500)",    dut->best_dist, 12500);

    // ---------- Arrival P2 = (5000, 4000), idx=2.  dist^2 = 4000^2 + 2000^2 = 20,000,000 ----------
    dut->arrival_valid = 1;
    dut->arrival_idx   = 2;
    dut->arrival_data  = pack2(5000, 4000);
    dut->eval();

    std::cout << "\n[4] Arrival P2=(5000,4000), idx=2.  Expected dist^2 = 20,000,000, no update:\n";
    check("cmp_dist for P2", dut->cmp_dist, 20000000);
    tick(dut, tfp);
    dut->arrival_valid = 0;
    dut->arrival_data  = 0;
    tick(dut, tfp);

    check("best_idx unchanged after P2 (stays at 1)",  dut->best_idx,  1);
    check("best_dist unchanged after P2 (stays 12500)", dut->best_dist, 12500);

    // ---------- Arrival P3 = (1050, 2025), idx=3.  dist^2 = 50^2 + 25^2 = 3125 ----------
    dut->arrival_valid = 1;
    dut->arrival_idx   = 3;
    dut->arrival_data  = pack2(1050, 2025);
    dut->eval();

    std::cout << "\n[5] Arrival P3=(1050,2025), idx=3.  Expected dist^2 = 3125, update:\n";
    check("cmp_dist for P3", dut->cmp_dist, 3125);
    tick(dut, tfp);
    dut->arrival_valid = 0;
    dut->arrival_data  = 0;
    tick(dut, tfp);

    check("best_idx after P3 (should update to 3)", dut->best_idx,  3);
    check("best_dist after P3 (should be 3125)",    dut->best_dist, 3125);

    // ---------- Done ----------
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    tfp->close();
    delete tfp;
    delete dut;

    std::cout << "\n=== Summary ===\n";
    std::cout << "  checks : " << n_checks << "\n";
    std::cout << "  passed : " << (n_checks - n_failures) << "\n";
    std::cout << "  failed : " << n_failures << "\n";
    std::cout << "  waveform: waves.vcd\n";

    if (n_failures == 0) {
        std::cout << "\nPASS\n";
        return 0;
    } else {
        std::cerr << "\nFAIL: see above; open waves.vcd in GTKWave to debug.\n";
        return 1;
    }
}
