// tb_distance_unit_folded.cpp
// =====================================================================
// Exhaustive testbench for the folded (MAC) distance_unit.sv.
//
// Updates from the pipelined version:
//   - Respects backpressure: waits for `in_ready` from the DUT before
//     asserting `in_valid` and driving new data.
//   - Drops `in_valid` immediately after data is accepted to prevent
//     double-feeding.
// =====================================================================

#include "Vdistance_unit.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

static constexpr int D         = 8;
static constexpr int IN_WIDTH  = 16;
static constexpr int VEC_WIDTH = D * IN_WIDTH;
static constexpr int VEC_WORDS = (VEC_WIDTH + 31) / 32;

static vluint64_t main_time = 0;
double sc_time_stamp() { return (double)main_time; }

static void pack_vec(uint32_t* dst, const int16_t* dims) {
    for (int w = 0; w < VEC_WORDS; w++) dst[w] = 0;
    for (int k = 0; k < D; k++) {
        uint32_t v = (uint32_t)(uint16_t)dims[k];
        int bit_pos = k * IN_WIDTH;
        int word    = bit_pos / 32;
        int shift   = bit_pos % 32;
        dst[word] |= v << shift;
        if (shift + IN_WIDTH > 32 && word + 1 < VEC_WORDS) {
            dst[word + 1] |= v >> (32 - shift);
        }
    }
}

// One "cycle" = one tick() = clk low then clk high.
static int g_cycle_count = 0;
static void tick(Vdistance_unit* dut, VerilatedVcdC* tfp) {
    dut->clk = 0; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
    dut->clk = 1; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
    g_cycle_count++;
}

static std::vector<int16_t> load_data_points(const std::string& path) {
    std::ifstream f(path);
    if (!f) { std::cerr << "ERROR: cannot open " << path << std::endl; std::exit(1); }
    std::vector<int16_t> data;
    std::string line;
    while (std::getline(f, line)) {
        auto nb = line.find_first_not_of(" \t\r\n");
        if (nb == std::string::npos) continue;
        line = line.substr(nb);
        if (line.empty() || line[0] == '/') continue;
        data.push_back((int16_t)(uint16_t)std::stoul(line, nullptr, 16));
    }
    return data;
}

struct Pair { int a; int b; uint64_t expected; };

static std::vector<Pair> load_pairs(const std::string& path) {
    std::ifstream f(path);
    if (!f) { std::cerr << "ERROR: cannot open " << path << std::endl; std::exit(1); }
    std::vector<Pair> pairs;
    std::string line;
    std::getline(f, line);
    while (std::getline(f, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> cells;
        while (std::getline(ss, cell, ',')) cells.push_back(cell);
        if (cells.size() < 3) continue;
        pairs.push_back({std::stoi(cells[0]), std::stoi(cells[1]),
                          std::stoull(cells[2])});
    }
    return pairs;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    std::string golden_dir = "./golden";
    int max_pairs = -1;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if      (a == "--golden"    && i + 1 < argc) golden_dir = argv[++i];
        else if (a == "--max-pairs" && i + 1 < argc) max_pairs  = std::stoi(argv[++i]);
    }
    std::cout << "Golden directory: " << golden_dir << std::endl;

    auto data  = load_data_points(golden_dir + "/data_points.mem");
    auto pairs = load_pairs       (golden_dir + "/dist_pairs_all.csv");
    const int N = (int)data.size() / D;
    std::cout << "Loaded " << data.size() << " values  =>  N=" << N
              << "  D=" << D << std::endl;
    std::cout << "Loaded " << pairs.size() << " pairs" << std::endl;
    if (max_pairs > 0 && (int)pairs.size() > max_pairs) {
        pairs.resize(max_pairs);
        std::cout << "  truncated to first " << max_pairs << std::endl;
    }

    Verilated::traceEverOn(true);
    auto* dut = new Vdistance_unit;
    auto* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves_distance_unit.vcd");

    // ---- Reset ----
    dut->rst_n    = 0;
    dut->in_valid = 0;
    dut->in_tag   = 0;
    memset((void*)dut->in_a, 0, VEC_WORDS * sizeof(uint32_t));
    memset((void*)dut->in_b, 0, VEC_WORDS * sizeof(uint32_t));
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    dut->rst_n = 1;
    
    // Tick once more so the DUT can assert in_ready after coming out of reset
    tick(dut, tfp);

    // ---- Drive all pairs, checking outputs as they emerge ----
    std::unordered_map<uint32_t, uint64_t> in_flight;
    int n_checks   = 0;
    int n_failures = 0;
    int first_input_cycle = -1;
    int first_output_cycle = -1;

    auto check_output = [&]() {
        if (!dut->out_valid) return;
        uint32_t tag = dut->out_tag;
        uint64_t got = dut->out_dist;
        auto it = in_flight.find(tag);
        if (it == in_flight.end()) {
            std::cerr << "FAIL cycle " << g_cycle_count
                      << ": output with unknown tag " << tag << std::endl;
            n_failures++;
            return;
        }
        n_checks++;
        if (got != it->second) {
            n_failures++;
            if (n_failures <= 10) {
                std::cerr << "FAIL tag " << tag
                          << "  got "      << got
                          << "  expected " << it->second << std::endl;
            }
        }
        in_flight.erase(it);
        if (first_output_cycle < 0) first_output_cycle = g_cycle_count;
    };

    for (size_t p = 0; p < pairs.size(); p++) {
        const Pair& pr = pairs[p];
        
        // Wait for DUT to be ready to accept new data
        while (!dut->in_ready) {
            dut->in_valid = 0; // Ensure we aren't asserting valid while waiting
            tick(dut, tfp);
            check_output();
        }

        uint32_t a_buf[VEC_WORDS], b_buf[VEC_WORDS];
        pack_vec(a_buf, &data[pr.a * D]);
        pack_vec(b_buf, &data[pr.b * D]);
        for (int w = 0; w < VEC_WORDS; w++) {
            dut->in_a[w] = a_buf[w];
            dut->in_b[w] = b_buf[w];
        }
        
        dut->in_valid = 1;
        dut->in_tag   = (uint32_t)(p & 0xFFFFFFFFu);

        if (first_input_cycle < 0) first_input_cycle = g_cycle_count;
        in_flight[(uint32_t)(p & 0xFFFFFFFFu)] = pr.expected;

        // Tick to register the transaction
        tick(dut, tfp);
        check_output();
        
        // Drop valid immediately so we don't hold it high on the next cycle
        dut->in_valid = 0;
    }

    // ---- Drain pipeline ----
    dut->in_valid = 0;
    memset((void*)dut->in_a, 0, VEC_WORDS * sizeof(uint32_t));
    memset((void*)dut->in_b, 0, VEC_WORDS * sizeof(uint32_t));

    int drain_cycles = 0;
    const int MAX_DRAIN = 200;
    while (!in_flight.empty() && drain_cycles < MAX_DRAIN) {
        tick(dut, tfp);
        check_output();
        drain_cycles++;
    }

    tfp->close();
    delete tfp;
    delete dut;

    // Latency calculation here will now represent the folded computation time 
    // rather than pipeline depth.
    int observed_latency = (first_output_cycle >= 0 && first_input_cycle >= 0)
                            ? (first_output_cycle - first_input_cycle)
                            : -1;

    std::cout << "\n=== Summary ===\n";
    std::cout << "  pairs checked:    " << n_checks << "\n";
    std::cout << "  failures:         " << n_failures << "\n";
    std::cout << "  unmatched at end: " << in_flight.size() << "\n";
    std::cout << "  observed latency: " << observed_latency << " cycles\n";
    std::cout << "  waveform: waves_distance_unit.vcd\n";

    if (n_failures == 0 && in_flight.empty()) {
        std::cout << "\nPASS\n";
        return 0;
    } else {
        std::cerr << "\nFAIL\n";
        return 1;
    }
}