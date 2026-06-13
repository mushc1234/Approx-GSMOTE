// tb_ann_wrapper.cpp
// =====================================================================
// Integration test for the ann_wrapper module.
//
// Plan:
//   1. Generate N=32 random 8-dim signed-16-bit points.
//   2. Build a behavioural AXI memory holding records of layout
//        {2B id, 2B pad, 16B data} = 20B/record = 5 AXI beats/record.
//   3. Drive the wrapper via AXI-Lite:
//        a. clear BRAM (CTRL = 0x2)
//        b. for each of T sort orders:
//             write N_POINTS, write POINTS_PTR, start (CTRL = 0x1),
//             poll STATUS.done, write-1-to-clear STATUS.done
//        c. read result MMIO for each point
//   4. Compare against brute-force NN. Single-pass exact match should
//      be modest; multi-pass aggregation across multiple sort orders
//      should improve coverage.
//
// The AXI-Lite traffic is generated in C++ via a tiny model that drives
// the slave's AW/W/B/AR/R channels directly. The AXI master read is
// served from a behavioural memory keyed by address.
// =====================================================================

#include "Vann_wrapper.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

static constexpr int W              = 4;
static constexpr int D              = 8;
static constexpr int IN_WIDTH       = 16;
static constexpr int IDX_WIDTH      = 16;
static constexpr int N_MAX_LOG2     = 8;
static constexpr int N_MAX          = 1 << N_MAX_LOG2;
static constexpr int N_POINTS       = 300;
static constexpr int RECORD_BYTES   = 4 + (D * IN_WIDTH / 8);   // 20
static constexpr int BEATS_PER_REC  = (RECORD_BYTES + 3) / 4;   // 5
static constexpr uint32_t POINTS_BASE = 0x1000'0000;

static vluint64_t main_time = 0;
double sc_time_stamp() { return (double)main_time; }

static int n_checks = 0, n_failures = 0;
static void check(const std::string& label, uint64_t got, uint64_t exp) {
    n_checks++;
    if (got != exp) {
        if (n_failures < 30) {
            std::cerr << "  FAIL: " << label
                      << "   got=" << got
                      << "   exp=" << exp << "\n";
        }
        n_failures++;
    }
}

// =====================================================================
// Behavioural AXI memory backing the master read port
//
// State is updated only post-edge (after the DUT's registers have
// settled). DUT inputs are driven pre-edge based on the current
// testbench state. This avoids the race where dropping arready
// mid-cycle would mean the DUT never sees arready=1 at the edge.
// =====================================================================
static std::map<uint32_t, uint32_t> ddr;        // byte addr -> 32-bit word
static int  ax_state = 0;                       // 0=idle, 1=delivering
static uint32_t ax_addr = 0;
static int  ax_beats_left = 0;

static void axi_drive_inputs(Vann_wrapper* dut) {
    if (ax_state == 0) {
        dut->m_axi_arready = 1;
        dut->m_axi_rvalid  = 0;
        dut->m_axi_rdata   = 0;
        dut->m_axi_rresp   = 0;
        dut->m_axi_rlast   = 0;
    } else {
        dut->m_axi_arready = 0;
        dut->m_axi_rvalid  = 1;
        auto it = ddr.find(ax_addr);
        dut->m_axi_rdata   = (it != ddr.end()) ? it->second : 0;
        dut->m_axi_rresp   = 0;
        dut->m_axi_rlast   = (ax_beats_left == 1) ? 1 : 0;
    }
}

// Activity counters for diagnostic
static uint64_t ax_ar_count = 0;
static uint64_t ax_r_count  = 0;

static void axi_capture_post_edge(bool arvalid_pre, bool rready_pre) {
    if (ax_state == 0) {
        // arready was driven=1 in this cycle's drive call; if the DUT
        // had arvalid asserted at the rising edge, we captured.
        if (arvalid_pre) {
            ax_state      = 1;
            ax_ar_count++;
        }
    } else {
        // rvalid was driven=1; if rready was asserted, a beat moved.
        if (rready_pre) {
            ax_addr       += 4;
            ax_beats_left -= 1;
            ax_r_count++;
            if (ax_beats_left == 0) {
                ax_state = 0;
            }
        }
    }
}

static void tick(Vann_wrapper* dut, VerilatedVcdC* tfp) {
    // Pre-edge: drive testbench-side AXI inputs based on testbench state.
    axi_drive_inputs(dut);

    dut->clk = 0; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;

    // Sample DUT signals exactly as they look at the rising edge.
    // These reflect the current registers, before the edge fires.
    bool     arvalid_pre = dut->m_axi_arvalid;
    bool     rready_pre  = dut->m_axi_rready;
    uint32_t araddr_pre  = dut->m_axi_araddr;
    uint8_t  arlen_pre   = dut->m_axi_arlen;

    dut->clk = 1; dut->eval();
    if (tfp) tfp->dump(main_time);
    main_time += 5;

    // Post-edge: capture AR fields BEFORE updating ax_state (the
    // address/length are only meaningful when arvalid was high).
    if (ax_state == 0 && arvalid_pre) {
        ax_addr       = araddr_pre;
        ax_beats_left = (int)arlen_pre + 1;
    }
    axi_capture_post_edge(arvalid_pre, rready_pre);
    axi_drive_inputs(dut);
    dut->eval();
}

// =====================================================================
// AXI-Lite host helpers (drive S_AXI_*)
// =====================================================================
static void axil_write(Vann_wrapper* dut, VerilatedVcdC* tfp,
                        uint32_t addr, uint32_t data) {
    dut->s_axi_awaddr  = addr;
    dut->s_axi_awvalid = 1;
    dut->s_axi_wdata   = data;
    dut->s_axi_wstrb   = 0xF;
    dut->s_axi_wvalid  = 1;
    dut->s_axi_bready  = 1;

    int safety = 0;
    while (!(dut->s_axi_awready && dut->s_axi_wready) && safety < 100) {
        tick(dut, tfp);
        safety++;
    }
    tick(dut, tfp);
    dut->s_axi_awvalid = 0;
    dut->s_axi_wvalid  = 0;

    // Wait for bvalid
    safety = 0;
    while (!dut->s_axi_bvalid && safety < 100) {
        tick(dut, tfp);
        safety++;
    }
    tick(dut, tfp);  // consume B
    dut->s_axi_bready = 0;
}

static uint32_t axil_read(Vann_wrapper* dut, VerilatedVcdC* tfp, uint32_t addr) {
    dut->s_axi_araddr  = addr;
    dut->s_axi_arvalid = 1;
    dut->s_axi_rready  = 1;

    int safety = 0;
    while (!dut->s_axi_arready && safety < 100) {
        tick(dut, tfp);
        safety++;
    }
    tick(dut, tfp);
    dut->s_axi_arvalid = 0;

    safety = 0;
    while (!dut->s_axi_rvalid && safety < 200) {
        tick(dut, tfp);
        safety++;
    }
    uint32_t v = (uint32_t)dut->s_axi_rdata;
    tick(dut, tfp);
    dut->s_axi_rready = 0;
    return v;
}

// =====================================================================
// Helpers: lay out points in DDR, pack record bytes
// =====================================================================
static void store_record(uint32_t base, uint16_t id,
                          const int16_t* dims) {
    // Beat 0: {pad[16], id[16]}
    ddr[base] = (uint32_t)id;
    // Beats 1..4: dims packed two per beat (little-endian)
    for (int k = 0; k < D / 2; k++) {
        uint16_t lo = (uint16_t)dims[2*k];
        uint16_t hi = (uint16_t)dims[2*k + 1];
        uint32_t w  = ((uint32_t)hi << 16) | (uint32_t)lo;
        ddr[base + 4 + k*4] = w;
    }
}

struct ResultEntry { bool valid; uint16_t res; uint16_t best; uint64_t dist; };

static ResultEntry read_result(Vann_wrapper* dut, VerilatedVcdC* tfp,
                                int idx) {
    uint32_t base = 0x1000 + idx * 16;
    uint32_t w0   = axil_read(dut, tfp, base + 0);
    uint32_t w1   = axil_read(dut, tfp, base + 4);
    uint32_t w2   = axil_read(dut, tfp, base + 8);
    uint32_t w3   = axil_read(dut, tfp, base + 12);
    ResultEntry r;
    r.valid = (w3 & 1u) != 0;
    r.res   = (uint16_t)(w2 & 0xFFFFu);
    r.best  = (uint16_t)((w2 >> 16) & 0xFFFFu);
    uint64_t dist_hi = (uint64_t)(w1 & 0xFFFFu);
    r.dist  = ((uint64_t)w0) | (dist_hi << 32);
    return r;
}

// =====================================================================
// Brute-force NN for reference
// =====================================================================
static uint64_t sqdist(const int16_t* a, const int16_t* b) {
    uint64_t s = 0;
    for (int k = 0; k < D; k++) {
        int64_t d = (int64_t)a[k] - (int64_t)b[k];
        s += (uint64_t)(d * d);
    }
    return s;
}

// =====================================================================
// Main
// =====================================================================
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    auto* dut = new Vann_wrapper;
    auto* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("waves_ann_wrapper.vcd");

    std::cout << "=== tb_ann_wrapper ===\n";
    std::cout << "  W=" << W << "  D=" << D << "  N=" << N_POINTS
              << "  N_MAX=" << N_MAX << "\n";
    std::cout << "  RECORD_BYTES=" << RECORD_BYTES
              << "  BEATS_PER_REC=" << BEATS_PER_REC << "\n";

    // -----------------------------------------------------------------
    // Generate test data
    // -----------------------------------------------------------------
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(-512, 512);
    std::vector<std::vector<int16_t>> points(N_POINTS, std::vector<int16_t>(D));
    for (int i = 0; i < N_POINTS; i++)
        for (int k = 0; k < D; k++)
            points[i][k] = (int16_t)dist(rng);

    // Brute-force NN
    std::vector<int> bf_nn(N_POINTS);
    std::vector<uint64_t> bf_dist(N_POINTS);
    for (int i = 0; i < N_POINTS; i++) {
        uint64_t best = ~0ULL;
        int bj = -1;
        for (int j = 0; j < N_POINTS; j++) {
            if (i == j) continue;
            uint64_t d = sqdist(points[i].data(), points[j].data());
            if (d < best) { best = d; bj = j; }
        }
        bf_nn[i]   = bj;
        bf_dist[i] = best;
    }

    // -----------------------------------------------------------------
    // Generate several sort orders using Random Projection LSH.
    // For each pass, we project all points onto a new random vector
    // and sort by the projected 1D values to preserve locality.
    // -----------------------------------------------------------------
    const int N_PASSES = 8;
    std::vector<std::vector<int>> orders(N_PASSES);
    
    // Standard normal distribution for generating isotropic random vectors
    std::normal_distribution<double> norm_dist(0.0, 1.0);

    for (int t = 0; t < N_PASSES; t++) {
        orders[t].resize(N_POINTS);
        for (int i = 0; i < N_POINTS; i++) orders[t][i] = i;
        
        // 1. Generate a random projection vector of dimension D
        std::vector<double> proj_vec(D);
        for (int k = 0; k < D; k++) {
            proj_vec[k] = norm_dist(rng);
        }
        
        // 2. Project each point onto the random vector
        std::vector<double> proj_vals(N_POINTS, 0.0);
        for (int i = 0; i < N_POINTS; i++) {
            for (int k = 0; k < D; k++) {
                proj_vals[i] += points[i][k] * proj_vec[k];
            }
        }
        
        // 3. Sort the sequence based on the projected 1D distances
        std::sort(orders[t].begin(), orders[t].end(), 
            [&proj_vals](int a, int b) {
                return proj_vals[a] < proj_vals[b];
            });
    }

    // -----------------------------------------------------------------
    // Reset DUT
    // -----------------------------------------------------------------
    dut->rst_n         = 0;
    dut->s_axi_awvalid = 0;
    dut->s_axi_wvalid  = 0;
    dut->s_axi_bready  = 0;
    dut->s_axi_arvalid = 0;
    dut->s_axi_rready  = 0;
    dut->m_axi_arready = 0;
    dut->m_axi_rvalid  = 0;
    dut->m_axi_rresp   = 0;
    dut->m_axi_rdata   = 0;
    dut->m_axi_rlast   = 0;
    for (int i = 0; i < 6; i++) tick(dut, tfp);
    dut->rst_n = 1;
    for (int i = 0; i < 4; i++) tick(dut, tfp);

    // -----------------------------------------------------------------
    // Smoke: write/read SCRATCH
    // -----------------------------------------------------------------
    axil_write(dut, tfp, 0x24, 0xC0DECAFE);
    uint32_t sc = axil_read(dut, tfp, 0x24);
    check("scratch readback", sc, 0xC0DECAFE);

    // -----------------------------------------------------------------
    // Clear BRAM
    // -----------------------------------------------------------------
    std::cout << "\n[clear]\n";
    axil_write(dut, tfp, 0x00, 0x02);  // CTRL.clear_bram = 1
    int safety = 0;
    while (safety < N_MAX + 200) {
        uint32_t st = axil_read(dut, tfp, 0x04);
        if ((st & (1u << 3)) == 0 && (st & 1u)) break;   // clear_busy=0 and idle=1
        safety++;
    }
    std::cout << "  clear settled after " << safety << " status reads\n";

    // -----------------------------------------------------------------
    // Multi-pass sequence
    // -----------------------------------------------------------------
    for (int t = 0; t < N_PASSES; t++) {
        // Lay this pass's points out in DDR in the sort order.
        ddr.clear();
        for (int pos = 0; pos < N_POINTS; pos++) {
            int pt = orders[t][pos];
            store_record(POINTS_BASE + pos * RECORD_BYTES,
                          (uint16_t)pt, points[pt].data());
        }
        std::cout << "\n[pass " << t << "]\n";

        axil_write(dut, tfp, 0x08, (uint32_t)N_POINTS);
        axil_write(dut, tfp, 0x0C, POINTS_BASE);
        axil_write(dut, tfp, 0x10, (uint32_t)t);
        axil_write(dut, tfp, 0x00, 0x01);   // start

        // Poll done
        safety = 0;
        const int LIMIT = 50000;
        while (safety < LIMIT) {
            uint32_t st = axil_read(dut, tfp, 0x04);
            if (st & (1u << 2)) break;   // done bit set
            // Every 200 polls, print state for diagnostic
            if ((safety % 200) == 199) {
                uint32_t hv  = axil_read(dut, tfp, 0x1C);
                uint32_t cyc = axil_read(dut, tfp, 0x14);
                std::cerr << "    poll " << safety
                          << " STATUS=0x" << std::hex << st << std::dec
                          << " hv=" << hv << " cyc=" << cyc
                          << " ar=" << ax_ar_count
                          << " r="  << ax_r_count << "\n";
            }
            safety++;
        }
        if (safety == LIMIT) {
            std::cerr << "  TIMEOUT pass " << t << "\n";
            n_failures++;
            break;
        }
        std::cout << "  done after " << safety << " polls\n";

        // Read perf counters
        uint32_t hv  = axil_read(dut, tfp, 0x1C);
        uint32_t imp = axil_read(dut, tfp, 0x20);
        std::cout << "  harvest_count=" << hv
                  << "  improvement_count=" << imp << "\n";

        // Acknowledge done
        axil_write(dut, tfp, 0x04, 0x04);  // W1C STATUS.done
    }

    // -----------------------------------------------------------------
    // Read all results and compare to brute force
    // -----------------------------------------------------------------
    int exact_match = 0;
    int valid_count = 0;
    int dist_ratio_le_2 = 0;
    for (int i = 0; i < N_POINTS; i++) {
        ResultEntry r = read_result(dut, tfp, i);
        if (r.valid) {
            valid_count++;
            check("entry " + std::to_string(i) + " resident_idx",
                  r.res, (uint64_t)i);
            // Compare best_dist to brute force
            if ((int)r.best == bf_nn[i] && r.dist == bf_dist[i]) {
                exact_match++;
            }
            if (bf_dist[i] != 0 && r.dist <= 2 * bf_dist[i]) {
                dist_ratio_le_2++;
            }
        }
    }

    std::cout << "\n=== Result vs brute force ===\n";
    std::cout << "  valid entries:    " << valid_count << " / " << N_POINTS << "\n";
    std::cout << "  exact NN matches: " << exact_match << " / " << N_POINTS << "\n";
    std::cout << "  dist <= 2x true:  " << dist_ratio_le_2 << " / " << N_POINTS << "\n";

    // Acceptance criteria (looser than RTL-internal tests; this is
    // about the wrapper sequencing being correct, not about the
    // algorithmic recall). Every point should be harvested at least
    // once across N_PASSES passes.
    check("every point harvested at least once",
          (uint64_t)valid_count, (uint64_t)N_POINTS);

    // Multi-pass aggregation should give *some* exact matches with 4 passes.
    if (exact_match == 0) {
        std::cerr << "  WARN: 0 exact NN matches; check engine integration\n";
    }

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