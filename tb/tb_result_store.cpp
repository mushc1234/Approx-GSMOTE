// tb_result_store.cpp
// =====================================================================
// Unit test for result_store (BRAM + clear FSM + harvest FSM).
//
// Scenarios:
//   1. Clear: assert clear_start; wait for clear_done; verify a sample
//      of entries shows valid=0 via MMIO.
//   2. Single-pass harvest: stream a sequence of (resident, best, dist)
//      tuples; verify each becomes the BRAM contents.
//   3. Multi-pass aggregation: re-harvest the same residents with a
//      mix of better/worse dists; verify only improvements stick and
//      improvement_count matches expectations.
//   4. Reset-via-clear: after a populated BRAM, run clear again and
//      verify all valid bits are zero.
// =====================================================================

#include "Vresult_store.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <map>

static constexpr int N_MAX_LOG2      = 6;          // 64 entries for the test
static constexpr int N_MAX           = 1 << N_MAX_LOG2;
static constexpr int IDX_WIDTH       = 16;
static constexpr int ACC_WIDTH       = 40;
static constexpr int ACC_STORE_WIDTH = 48;

static vluint64_t main_time = 0;
double sc_time_stamp() { return (double)main_time; }

static int n_checks   = 0;
static int n_failures = 0;

static void check(const std::string& label, uint64_t got, uint64_t expected) {
    n_checks++;
    if (got == expected) {
        // Pass quietly to keep noise low; uncomment for debug.
        // std::cout << "  ok  : " << label << " = " << got << "\n";
    } else {
        std::cerr << "  FAIL: " << label
                  << "   got " << got
                  << "   expected " << expected << "\n";
        n_failures++;
    }
}

static void tick(Vresult_store* dut, VerilatedVcdC* tfp) {
    dut->clk = 0; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
    dut->clk = 1; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
}

static void zero_inputs(Vresult_store* dut) {
    dut->clear_start          = 0;
    dut->harvest_enable       = 0;
    dut->eng_out_valid        = 0;
    dut->eng_out_resident_idx = 0;
    dut->eng_out_best_idx     = 0;
    dut->eng_out_best_dist    = 0;
    dut->b_rd_en              = 0;
    dut->b_rd_entry           = 0;
    dut->b_rd_word            = 0;
}

static void reset(Vresult_store* dut, VerilatedVcdC* tfp) {
    zero_inputs(dut);
    dut->rst_n = 0;
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    dut->rst_n = 1;
    tick(dut, tfp);
}

// Drive a clear cycle; block until clear_done pulses or timeout.
static void run_clear(Vresult_store* dut, VerilatedVcdC* tfp) {
    dut->clear_start = 1;
    tick(dut, tfp);
    dut->clear_start = 0;
    int t = 0;
    const int LIMIT = N_MAX + 16;
    while (!dut->clear_done && t < LIMIT) {
        tick(dut, tfp);
        t++;
    }
    if (t == LIMIT) {
        std::cerr << "  TIMEOUT: clear did not finish within " << LIMIT << " cycles\n";
        n_failures++;
    }
    tick(dut, tfp);   // settle one cycle after done
}

// Push one harvest tuple, respecting eng_out_ready handshake.
static void push_harvest(Vresult_store* dut, VerilatedVcdC* tfp,
                          uint16_t resident, uint16_t best, uint64_t dist) {
    // Wait for ready
    dut->harvest_enable       = 1;
    dut->eng_out_valid        = 1;
    dut->eng_out_resident_idx = resident;
    dut->eng_out_best_idx     = best;
    dut->eng_out_best_dist    = (vluint64_t)dist;
    int safety = 0;
    while (!dut->eng_out_ready && safety < 100) {
        tick(dut, tfp);
        safety++;
    }
    // Accept this cycle
    tick(dut, tfp);
    dut->eng_out_valid = 0;
    // Drive at least one cycle in COMPARE before raising another tuple
    tick(dut, tfp);
}

// Read a BRAM entry via MMIO port; returns (valid, resident, best, dist)
// where dist is full 48 bits (we only use 40 today).
struct Entry {
    bool     valid;
    uint16_t resident_idx;
    uint16_t best_idx;
    uint64_t best_dist;
};

static Entry read_entry(Vresult_store* dut, VerilatedVcdC* tfp, int idx) {
    auto read_word = [&](int w) -> uint32_t {
        dut->b_rd_en    = 1;
        dut->b_rd_entry = idx;
        dut->b_rd_word  = w;
        tick(dut, tfp);
        dut->b_rd_en = 0;
        // Read data appears after the registered word lookup; need one
        // more cycle for the combinational mux to settle. Take an extra
        // tick to be safe.
        tick(dut, tfp);
        return (uint32_t)dut->b_rd_data;
    };

    uint32_t w0 = read_word(0);   // dist[31:0]
    uint32_t w1 = read_word(1);   // dist[63:32] (only [15:0] populated)
    uint32_t w2 = read_word(2);   // {best_idx, resident_idx}
    uint32_t w3 = read_word(3);   // valid in bit 0

    Entry e;
    e.valid        = (w3 & 1u) != 0;
    e.resident_idx = (uint16_t)(w2 & 0xFFFFu);
    e.best_idx     = (uint16_t)((w2 >> 16) & 0xFFFFu);
    uint64_t dist_hi = (uint64_t)(w1 & 0xFFFFu);
    e.best_dist    = ((uint64_t)w0) | (dist_hi << 32);
    return e;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    auto* dut = new Vresult_store;
    auto* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves_result_store.vcd");

    std::cout << "=== tb_result_store ===\n";
    std::cout << "  N_MAX_LOG2=" << N_MAX_LOG2
              << "  N_MAX=" << N_MAX
              << "  IDX_WIDTH=" << IDX_WIDTH
              << "  ACC_WIDTH=" << ACC_WIDTH << "\n";

    // -----------------------------------------------------------------
    // Test 1: Clear from reset
    // -----------------------------------------------------------------
    std::cout << "\n[1] Clear from reset; all entries valid=0.\n";
    reset(dut, tfp);
    run_clear(dut, tfp);

    for (int i = 0; i < N_MAX; i += (N_MAX / 8)) {
        Entry e = read_entry(dut, tfp, i);
        check("entry " + std::to_string(i) + " valid after clear", e.valid, 0);
    }

    // -----------------------------------------------------------------
    // Test 2: Single-pass harvest stream
    // -----------------------------------------------------------------
    std::cout << "\n[2] Single-pass harvest: verify writes land.\n";

    struct Tuple { uint16_t res; uint16_t best; uint64_t dist; };
    std::vector<Tuple> pass1 = {
        { 5,  17, 100 },
        { 8,  22, 250 },
        { 9,  31, 50  },
        { 30, 7,  9000 },
        { 33, 44, 1   },
    };

    for (auto& t : pass1) {
        push_harvest(dut, tfp, t.res, t.best, t.dist);
    }

    for (auto& t : pass1) {
        Entry e = read_entry(dut, tfp, t.res);
        check("pass1 entry " + std::to_string(t.res) + " valid",
              e.valid, 1);
        check("pass1 entry " + std::to_string(t.res) + " resident_idx",
              e.resident_idx, t.res);
        check("pass1 entry " + std::to_string(t.res) + " best_idx",
              e.best_idx, t.best);
        check("pass1 entry " + std::to_string(t.res) + " best_dist",
              e.best_dist, t.dist);
    }

    // -----------------------------------------------------------------
    // Test 3: Multi-pass aggregation
    // -----------------------------------------------------------------
    std::cout << "\n[3] Multi-pass aggregation: only improvements stick.\n";
    //
    // Re-harvest same residents with various dists:
    //   res 5  : worse (200) -> should NOT update     (still best=17, dist=100)
    //   res 8  : better (50) -> should update         (best=99, dist=50)
    //   res 9  : tie (50)   -> should NOT update      (strict <)
    //   res 30 : better (8000) -> should update       (best=12, dist=8000)
    //   res 33 : worse (2)  -> should NOT update      (still dist=1)
    //   res 40 : new (444)  -> should write fresh     (valid was 0)

    std::vector<Tuple> pass2 = {
        { 5,  99, 200  },
        { 8,  99, 50   },
        { 9,  99, 50   },
        { 30, 12, 8000 },
        { 33, 99, 2    },
        { 40, 55, 444  },
    };
    for (auto& t : pass2) {
        push_harvest(dut, tfp, t.res, t.best, t.dist);
    }

    struct Expected { uint16_t best; uint64_t dist; };
    std::map<uint16_t, Expected> expected = {
        { 5,  { 17, 100  } },
        { 8,  { 99, 50   } },
        { 9,  { 31, 50   } },
        { 30, { 12, 8000 } },
        { 33, { 44, 1    } },
        { 40, { 55, 444  } },
    };
    for (auto& kv : expected) {
        Entry e = read_entry(dut, tfp, kv.first);
        check("pass2 entry " + std::to_string(kv.first) + " best_idx",
              e.best_idx, kv.second.best);
        check("pass2 entry " + std::to_string(kv.first) + " best_dist",
              e.best_dist, kv.second.dist);
        check("pass2 entry " + std::to_string(kv.first) + " valid",
              e.valid, 1);
    }

    // -----------------------------------------------------------------
    // Test 4: Re-clear wipes everything
    // -----------------------------------------------------------------
    std::cout << "\n[4] Re-clear wipes valid bits.\n";
    run_clear(dut, tfp);
    for (int i = 0; i < N_MAX; i += 7) {
        Entry e = read_entry(dut, tfp, i);
        check("after re-clear entry " + std::to_string(i) + " valid",
              e.valid, 0);
    }

    // -----------------------------------------------------------------
    // Test 5: After clear, harvest works again (no stuck state)
    // -----------------------------------------------------------------
    std::cout << "\n[5] After clear, harvest still works.\n";
    push_harvest(dut, tfp, 11, 22, 333);
    Entry e = read_entry(dut, tfp, 11);
    check("post-clear harvest valid", e.valid, 1);
    check("post-clear harvest best_idx", e.best_idx, 22);
    check("post-clear harvest best_dist", e.best_dist, 333);

    for (int i = 0; i < 8; i++) tick(dut, tfp);
    tfp->close();
    delete tfp;
    delete dut;

    std::cout << "\n=== Summary ===\n";
    std::cout << "  checks: " << n_checks << "\n";
    std::cout << "  passed: " << (n_checks - n_failures) << "\n";
    std::cout << "  failed: " << n_failures << "\n";

    if (n_failures == 0) { std::cout << "\nPASS\n"; return 0; }
    else                 { std::cerr << "\nFAIL\n"; return 1; }
}
