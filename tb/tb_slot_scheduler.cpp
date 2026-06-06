// tb_slot_scheduler.cpp
// =====================================================================
// Testbench for slot_scheduler.sv. Tests the state machine in
// isolation by mocking slot state from the C++ side -- the scheduler
// reads slot_resident_idx[], slot_best_idx[], slot_best_dist[] as
// inputs, and we just drive them with values that let us verify the
// scheduler is selecting the right slot and emitting the right tuples.
//
// Three test sections:
//
//   1. Basic eviction sequence: drive arrivals, observe that one
//      eviction happens per EVICT_INTERVAL=1 arrivals, that harvest
//      tuples come out in round-robin order, that load_valid is
//      one-hot on the expected slot.
//
//   2. Drain timing: verify the scheduler stalls in_ready for
//      LATENCY cycles between arriving and harvesting, so in-flight
//      pipelined comparisons have time to settle.
//
//   3. Flush: at end of stream, assert flush and verify all W slots
//      get harvested without further loads, then flush_done asserts.
//
// Parameters used here match the DUT instantiation in the SV (W=4
// for quicker tests; LATENCY=5 to match D=8 pipelined distance unit).
// =====================================================================

#include "Vslot_scheduler.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

// Must match the SV parameters
static constexpr int W       = 4;
static constexpr int LATENCY = 5;

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
                  << "   got " << got
                  << "   expected " << expected << "\n";
        n_failures++;
    }
}

static void tick(Vslot_scheduler* dut, VerilatedVcdC* tfp) {
    dut->clk = 0; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
    dut->clk = 1; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
}

// Initialize all mock slot state so the scheduler reads stable values.
// Each slot's "resident_idx" is set to (100 + slot_idx), and its
// running-best is some distinct sentinel. Lets the test verify that
// harvest tuples emerge in the correct round-robin order with the
// values that *should* belong to the slot being evicted.
static void init_mock_slots(Vslot_scheduler* dut) {
    for (int s = 0; s < W; s++) {
        dut->slot_resident_idx[s] = 100 + s;
        dut->slot_best_idx    [s] = 200 + s;
        dut->slot_best_dist   [s] = 0x1000ULL + s;
    }
}

static void zero_inputs(Vslot_scheduler* dut) {
    dut->in_valid        = 0;
    dut->in_arrival_idx  = 0;
    dut->in_arrival_data = 0;
    dut->flush           = 0;
    dut->out_ready       = 1;   // host is always ready to receive
    init_mock_slots(dut);
}

static void reset(Vslot_scheduler* dut, VerilatedVcdC* tfp) {
    zero_inputs(dut);
    dut->rst_n = 0;
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    dut->rst_n = 1;
    tick(dut, tfp);
}

// Find which bit in a W-bit vector is set; -1 if none. Used to check
// the one-hot slot_load_valid.
static int onehot_idx(uint32_t bits) {
    int found = -1;
    for (int i = 0; i < 32; i++) {
        if (bits & (1u << i)) {
            if (found >= 0) return -2;   // multiple bits set: invalid
            found = i;
        }
    }
    return found;
}

// =====================================================================
// Test 1 -- basic eviction sequence
// =====================================================================
static void test1_eviction(Vslot_scheduler* dut, VerilatedVcdC* tfp) {
    std::cout << "\n========== TEST 1: basic eviction sequence ==========\n";
    reset(dut, tfp);

    // Drive W arrivals; expect W evictions in round-robin order
    // starting at slot 0.
    for (int arrival = 0; arrival < W; arrival++) {
        std::cout << "\n[arrival " << arrival << "]\n";

        // Wait for scheduler to be ready to accept arrival
        int wait_cycles = 0;
        while (!dut->in_ready && wait_cycles < 100) {
            tick(dut, tfp);
            wait_cycles++;
        }
        check("in_ready asserted before arrival", dut->in_ready, 1);

        // Drive the arrival
        dut->in_valid        = 1;
        dut->in_arrival_idx  = 1000 + arrival;
        dut->in_arrival_data = 0xABCD0000u + arrival;
        dut->eval();
        // Scheduler should broadcast to all slots this same cycle
        check("slot_arrival_valid during accept", dut->slot_arrival_valid, 1);
        check("slot_arrival_idx broadcast",       dut->slot_arrival_idx,
              (uint64_t)(1000 + arrival));

        tick(dut, tfp);   // arrival latched
        dut->in_valid        = 0;
        dut->in_arrival_data = 0;
        dut->in_arrival_idx  = 0;

        // After arrival, scheduler enters DRAIN. in_ready should be 0
        // for LATENCY cycles, then HARVEST, then LOAD, then STREAM.
        // Count cycles until harvest output appears.
        int harvest_cycle = -1;
        int load_cycle    = -1;
        for (int cy = 0; cy < 30; cy++) {
            tick(dut, tfp);
            if (dut->out_valid && harvest_cycle < 0) {
                harvest_cycle = cy;
                // Expected slot to harvest: round-robin, so == arrival % W
                int expected_slot = arrival % W;
                check("harvest out_resident_idx",
                      dut->out_resident_idx, (uint64_t)(100 + expected_slot));
                check("harvest out_best_idx",
                      dut->out_best_idx, (uint64_t)(200 + expected_slot));
                check("harvest out_best_dist",
                      dut->out_best_dist, (uint64_t)(0x1000ULL + expected_slot));
            }
            uint32_t lv = (uint32_t)dut->slot_load_valid;
            if (lv != 0 && load_cycle < 0) {
                load_cycle = cy;
                int loaded_slot = onehot_idx(lv);
                int expected_slot = arrival % W;
                check("slot_load_valid one-hot",  (loaded_slot >= 0), 1);
                check("slot_load_valid on correct slot",
                      (uint64_t)loaded_slot, (uint64_t)expected_slot);
                check("slot_load_idx is the arrival idx",
                      dut->slot_load_idx, (uint64_t)(1000 + arrival));
            }
            if (load_cycle >= 0 && dut->in_ready) break;
        }
        check("harvest happened", (harvest_cycle >= 0), 1);
        check("load happened",    (load_cycle    >= 0), 1);
        // Harvest should come BEFORE load
        if (harvest_cycle >= 0 && load_cycle >= 0) {
            check("harvest precedes load", (harvest_cycle < load_cycle), 1);
        }
    }
}

// =====================================================================
// Test 2 -- drain timing
// =====================================================================
static void test2_drain(Vslot_scheduler* dut, VerilatedVcdC* tfp) {
    std::cout << "\n========== TEST 2: drain timing ==========\n";
    reset(dut, tfp);

    // Drive one arrival
    dut->in_valid        = 1;
    dut->in_arrival_idx  = 42;
    dut->in_arrival_data = 0xDEADBEEFu;
    tick(dut, tfp);
    dut->in_valid        = 0;
    dut->in_arrival_data = 0;
    dut->in_arrival_idx  = 0;

    // Now check: in_ready should be deasserted for LATENCY cycles,
    // then harvest should occur. This verifies the drain budget.
    int cycles_stalled = 0;
    for (int cy = 0; cy < LATENCY + 5; cy++) {
        if (!dut->out_valid && !dut->in_ready) cycles_stalled++;
        if (dut->out_valid) {
            std::cout << "  harvest appeared at cycle " << cy
                      << " (after " << cycles_stalled
                      << " stalled cycles)\n";
            // We expect at least LATENCY cycles of stall before harvest
            // (the scheduler counts drain_cnt down from LATENCY).
            check("stall covers at least LATENCY cycles",
                  (cycles_stalled >= LATENCY), 1);
            break;
        }
        tick(dut, tfp);
    }
}

// =====================================================================
// Test 3 -- flush sequence
// =====================================================================
static void test3_flush(Vslot_scheduler* dut, VerilatedVcdC* tfp) {
    std::cout << "\n========== TEST 3: flush sequence ==========\n";
    reset(dut, tfp);

    // Drive a couple of arrivals so the round-robin pointer advances
    for (int i = 0; i < 2; i++) {
        // Wait for ready
        int waits = 0;
        while (!dut->in_ready && waits < 100) { tick(dut, tfp); waits++; }
        dut->in_valid        = 1;
        dut->in_arrival_idx  = 500 + i;
        dut->in_arrival_data = 0xCAFE0000u + i;
        tick(dut, tfp);
        dut->in_valid = 0;
    }

    // Wait for the scheduler to settle back to STREAM (in_ready high)
    int settle = 0;
    while (!dut->in_ready && settle < 50) { tick(dut, tfp); settle++; }

    // Now assert flush
    std::cout << "\n[asserting flush]\n";
    dut->flush = 1;
    tick(dut, tfp);

    // Expect W harvest beats, then flush_done
    int harvests_seen = 0;
    std::vector<uint64_t> harvested_residents;
    for (int cy = 0; cy < 100; cy++) {
        if (dut->out_valid && dut->out_ready) {
            harvested_residents.push_back(dut->out_resident_idx);
            harvests_seen++;
            std::cout << "  flush harvest " << harvests_seen
                      << ": resident_idx = " << dut->out_resident_idx << "\n";
        }
        if (dut->flush_done) {
            std::cout << "  flush_done asserted at cycle " << cy << "\n";
            break;
        }
        tick(dut, tfp);
    }
    check("flush harvested W slots", (uint64_t)harvests_seen, (uint64_t)W);
    check("flush_done asserted", dut->flush_done, 1);

    // All W slots should be in the harvest set (in some round-robin order)
    bool all_present = true;
    for (int s = 0; s < W; s++) {
        bool found = false;
        for (auto r : harvested_residents) {
            if (r == (uint64_t)(100 + s)) { found = true; break; }
        }
        if (!found) all_present = false;
    }
    check("all W slots appear in flush output", all_present, 1);

    // Deassert flush and verify scheduler returns to STREAM
    dut->flush = 0;
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    check("scheduler returns to ready after flush", dut->in_ready, 1);
}

// =====================================================================
// Main
// =====================================================================
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    auto* dut = new Vslot_scheduler;
    auto* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves_slot_scheduler.vcd");

    std::cout << "=== tb_slot_scheduler ===   W=" << W
              << "  LATENCY=" << LATENCY << "\n";

    test1_eviction(dut, tfp);
    test2_drain   (dut, tfp);
    test3_flush   (dut, tfp);

    for (int i = 0; i < 4; i++) tick(dut, tfp);

    tfp->close();
    delete tfp;
    delete dut;

    std::cout << "\n=== Summary ===\n";
    std::cout << "  checks : " << n_checks << "\n";
    std::cout << "  passed : " << (n_checks - n_failures) << "\n";
    std::cout << "  failed : " << n_failures << "\n";

    if (n_failures == 0) { std::cout << "\nPASS\n"; return 0; }
    else                 { std::cerr << "\nFAIL\n"; return 1; }
}
