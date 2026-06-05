// tb_distance_unit.cpp
// =====================================================================
// Exhaustive testbench for distance_unit.sv.
//
// Loads data_points.mem and dist_pairs_all.csv from the golden generator
// (see generate_golden_vectors.py), drives every (i, j) pair through the
// distance unit, and checks the result against the integer-exact golden.
//
// Usage:
//   ./Vdistance_unit                              # uses ./golden
//   ./Vdistance_unit --golden ../golden_other     # different dir
//   ./Vdistance_unit --max-pairs 100              # truncate (debug)
//   ./Vdistance_unit --stop-on-fail               # stop at first failure
//
// The DUT's D parameter and the golden's D MUST match. They are both
// set to 8 here for the default small bring-up case. To use the larger
// deployment-shape golden (D=32), change D in both this file and
// rtl/distance_unit.sv parameter override.
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
#include <vector>

// Test parameters -- must match the DUT
static constexpr int D         = 8;
static constexpr int IN_WIDTH  = 16;
static constexpr int VEC_WIDTH = D * IN_WIDTH;          // 128 for D=8
static constexpr int VEC_WORDS = (VEC_WIDTH + 31) / 32; // 4 for D=8

static vluint64_t main_time = 0;
double sc_time_stamp() { return (double)main_time; }

// ----------------------------------------------------------------------
// Pack D signed 16-bit dimensions into the packed bus's uint32_t-array
// representation. Dim k goes into bits [(k+1)*IN_WIDTH-1 : k*IN_WIDTH]
// of the bus, which becomes a particular bit range of a particular word.
// ----------------------------------------------------------------------
static void pack_vec(uint32_t* dst, const int16_t* dims) {
    for (int w = 0; w < VEC_WORDS; w++) dst[w] = 0;
    for (int k = 0; k < D; k++) {
        uint32_t v = (uint32_t)(uint16_t)dims[k];   // zero-extended bit pattern
        int bit_pos = k * IN_WIDTH;
        int word    = bit_pos / 32;
        int shift   = bit_pos % 32;
        dst[word] |= v << shift;
        if (shift + IN_WIDTH > 32 && word + 1 < VEC_WORDS) {
            dst[word + 1] |= v >> (32 - shift);
        }
    }
}

static void tick(Vdistance_unit* dut, VerilatedVcdC* tfp) {
    dut->clk = 0; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
    dut->clk = 1; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;
}

// ----------------------------------------------------------------------
// Golden loaders
// ----------------------------------------------------------------------
static std::vector<int16_t> load_data_points(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "ERROR: cannot open " << path << std::endl;
        std::exit(1);
    }
    std::vector<int16_t> data;
    std::string line;
    while (std::getline(f, line)) {
        auto nb = line.find_first_not_of(" \t\r\n");
        if (nb == std::string::npos) continue;
        line = line.substr(nb);
        if (line.empty() || line[0] == '/') continue;
        uint32_t v = std::stoul(line, nullptr, 16);
        data.push_back((int16_t)(uint16_t)v);    // sign-extend via cast chain
    }
    return data;
}

struct Pair { int a; int b; uint64_t expected; };

static std::vector<Pair> load_pairs(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "ERROR: cannot open " << path << std::endl;
        std::exit(1);
    }
    std::vector<Pair> pairs;
    std::string line;
    std::getline(f, line);  // header
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

// ----------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    std::string golden_dir = "./golden";
    int max_pairs = -1;
    bool stop_on_fail = false;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if      (a == "--golden"       && i + 1 < argc) golden_dir   = argv[++i];
        else if (a == "--max-pairs"    && i + 1 < argc) max_pairs    = std::stoi(argv[++i]);
        else if (a == "--stop-on-fail")                  stop_on_fail = true;
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
        std::cout << "  truncated to first " << max_pairs << " for this run" << std::endl;
    }

    Verilated::traceEverOn(true);
    Vdistance_unit* dut = new Vdistance_unit;
    VerilatedVcdC*  tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves_distance_unit.vcd");

    // Reset
    dut->rst_n    = 0;
    dut->in_valid = 0;
    dut->in_tag   = 0;
    memset((void*)dut->in_a, 0, VEC_WORDS * sizeof(uint32_t));
    memset((void*)dut->in_b, 0, VEC_WORDS * sizeof(uint32_t));
    for (int i = 0; i < 4; i++) tick(dut, tfp);
    dut->rst_n = 1;
    tick(dut, tfp);

    // Drive all pairs
    int n_checks = 0, n_failures = 0;
    for (size_t p = 0; p < pairs.size(); p++) {
        const Pair& pr = pairs[p];
        uint32_t a_buf[VEC_WORDS], b_buf[VEC_WORDS];
        pack_vec(a_buf, &data[pr.a * D]);
        pack_vec(b_buf, &data[pr.b * D]);

        for (int w = 0; w < VEC_WORDS; w++) {
            dut->in_a[w] = a_buf[w];
            dut->in_b[w] = b_buf[w];
        }
        dut->in_valid = 1;
        dut->in_tag   = (uint32_t)(p & 0xFFFFFFFFu);
        dut->eval();                              // combinational settle

        // Check
        n_checks++;
        const uint64_t got = dut->out_dist;
        if (got != pr.expected) {
            n_failures++;
            if (n_failures <= 10 || stop_on_fail) {
                std::cerr << "FAIL pair " << p
                          << "  (" << pr.a << "," << pr.b << ")"
                          << "  got "      << got
                          << "  expected " << pr.expected
                          << "  diff " << ((int64_t)got - (int64_t)pr.expected)
                          << std::endl;
            }
            if (stop_on_fail) break;
        }
        if (dut->out_valid != 1) {
            n_failures++;
            std::cerr << "FAIL pair " << p << "  out_valid not asserted" << std::endl;
        }
        if (dut->out_tag != (uint32_t)(p & 0xFFFFFFFFu)) {
            n_failures++;
            std::cerr << "FAIL pair " << p << "  out_tag mismatch"
                      << "  got "      << dut->out_tag
                      << "  expected " << (uint32_t)(p & 0xFFFFFFFFu)
                      << std::endl;
        }

        tick(dut, tfp);
    }

    dut->in_valid = 0;
    for (int i = 0; i < 4; i++) tick(dut, tfp);

    tfp->close();
    delete tfp;
    delete dut;

    std::cout << "\n=== Summary ===\n";
    std::cout << "  pairs checked: " << n_checks << "\n";
    std::cout << "  failures:      " << n_failures << "\n";
    std::cout << "  waveform: waves_distance_unit.vcd\n";

    if (n_failures == 0) {
        std::cout << "\nPASS\n";
        return 0;
    } else {
        std::cerr << "\nFAIL\n";
        return 1;
    }
}
