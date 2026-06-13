// main.cpp -- GSMOTE on MicroBlaze
// =====================================================================
// Bare-metal Vitis app driving gsmote_mb.hpp. Embeds a small synthetic
// dataset, runs gsmote_fit_resample once, prints class counts and
// elapsed time over UART via xil_printf.
//
// Build setup in Vitis:
//   - Application type:  Empty C++ Application
//   - Compiler flags:    -O2 -fno-exceptions -fno-rtti
//                        (and FPU flags should be auto-picked from BSP)
//   - Linker script:     bump _HEAP_SIZE to 0x100000 (1 MB) and place
//                        .heap in the MIG/DDR region.
//   - Includes:          gsmote_mb.hpp in the same src/ dir
//
// Once this works, swap the procedural dataset for a CSV-generated one
// via csv_to_c.py (a 10k KEEL set will fit comfortably in DDR).
// =====================================================================

#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xiltimer.h"

#include "gsmote_mb.hpp"

#include <cstddef>
#include <cstdint>

// ---------------------------------------------------------------------
// Synthetic dataset config -- start small to validate toolchain.
// Once green, bump N_MAJORITY/N_MINORITY_* to stress the kNN, OR swap
// to KEEL data via csv_to_c.py.
//
// Memory layout note: we DO NOT use a static `dataset_X[]` array here.
// Instead, we generate directly into the gsmote::Dataset's vectors,
// which live on the heap. With the heap mapped to DDR via lscript.ld,
// this means the dataset (and all algorithm scratch) lives in DDR --
// scales to 10k+ samples without BRAM concerns.
// ---------------------------------------------------------------------
static constexpr int N_FEATURES   = 8;
static constexpr int N_MAJORITY   = 1000;
static constexpr int N_MINORITY_A = 100;
static constexpr int N_TOTAL      = N_MAJORITY + N_MINORITY_A;

// Fill the input Dataset's vectors directly -- no intermediate static
// array. The vectors are on the heap, which is in DDR.
static void generate_dataset(gsmote::Dataset& ds) {
    ds.n_features = N_FEATURES;
    ds.n_samples  = N_TOTAL;
    ds.X.resize(static_cast<std::size_t>(N_TOTAL) * N_FEATURES);
    ds.y.resize(static_cast<std::size_t>(N_TOTAL));

    gsmote::XorShift64 rng(12345);
    std::size_t row = 0;

    // Majority cluster: centered at origin
    for (int i = 0; i < N_MAJORITY; ++i) {
        for (int k = 0; k < N_FEATURES; ++k) {
            ds.X[row * N_FEATURES + k] = rng.normal01();
        }
        ds.y[row] = 0;
        ++row;
    }
    // Minority A: well-separated cluster
    for (int i = 0; i < N_MINORITY_A; ++i) {
        ds.X[row * N_FEATURES + 0] = 4.0f + rng.normal01();
        ds.X[row * N_FEATURES + 1] = 4.0f + rng.normal01();
        ds.X[row * N_FEATURES + 2] = rng.normal01();
        ds.X[row * N_FEATURES + 3] = rng.normal01();
        ds.y[row] = 1;
        ++row;
    }
}

// xil_printf can't format floats. For sanity-checking a few values we
// print them as fixed-point millis: float * 1000 as a signed int.
static void print_float_x1000(const char* lbl, float f) {
    int v = (int)(f * 1000.0f);
    xil_printf("%s = %d.%03d (x1000)\r\n", lbl, v / 1000, v < 0 ? -v % 1000 : v % 1000);
}

// Print class label -> count, up to MAX_CLASSES distinct labels.
static constexpr int MAX_CLASSES = 16;
static void print_class_counts(const char* tag,
                               const int32_t* y, std::size_t n) {
    int32_t labels[MAX_CLASSES] = {0};
    int     counts[MAX_CLASSES] = {0};
    int     n_classes = 0;
    for (std::size_t i = 0; i < n; ++i) {
        int found = -1;
        for (int c = 0; c < n_classes; ++c) {
            if (labels[c] == y[i]) { found = c; break; }
        }
        if (found < 0) {
            if (n_classes >= MAX_CLASSES) continue;
            labels[n_classes] = y[i];
            counts[n_classes] = 1;
            ++n_classes;
        } else {
            counts[found]++;
        }
    }
    xil_printf("%s N=%d classes={", tag, (int)n);
    for (int c = 0; c < n_classes; ++c) {
        xil_printf("%s%d:%d", (c == 0) ? "" : ", ",
                   (int)labels[c], counts[c]);
    }
    xil_printf("}\r\n");
}

int main(void) {
    // init_platform();
    xil_printf("\r\n=== GSMOTE on MicroBlaze ===\r\n");
    xil_printf("D=%d, N=%d (synthetic dataset)\r\n",
               N_FEATURES, N_TOTAL);

    // Build input dataset directly in heap-backed vectors (which live
    // in DDR per the linker script).
    gsmote::Dataset in;
    generate_dataset(in);
    print_float_x1000("in.X[0]", in.X[0]);  // sanity check FPU works

    print_class_counts("input :", in.y.data(), in.n_samples);

    gsmote::Params p;
    p.k_neighbors        = 5;
    p.random_seed        = 42;
    p.selection_strategy = gsmote::SelectionStrategy::Combined;
    p.truncation_factor  = 0.0f;
    p.deformation_factor = 0.0f;

    XTime t0, t1;
    XTime_GetTime(&t0);

    gsmote::Dataset out;
    bool ok = gsmote::gsmote_fit_resample(in, p, out);

    XTime_GetTime(&t1);

    if (!ok) {
        xil_printf("ERROR: gsmote_fit_resample failed (bad args)\r\n");
        //cleanup_platform();
        return -1;
    }

    print_class_counts("output:", out.y.data(), out.n_samples);

    // XTime ticks are at COUNTS_PER_SECOND. Convert to microseconds.
    // For MicroBlaze with the standard BSP, COUNTS_PER_SECOND ==
    // (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 2) typically, but using the
    // macro is portable. If COUNTS_PER_SECOND isn't defined the BSP
    // is missing a timer -- add axi_timer_0 to the block design.
    uint64_t ticks = (uint64_t)(t1 - t0);
    uint32_t us = (uint32_t)((ticks * 1000000ULL) / COUNTS_PER_SECOND);
    uint32_t ms = us / 1000;

    xil_printf("synthetic samples added: %d\r\n",
               (int)(out.n_samples - in.n_samples));
    xil_printf("elapsed: %u us (%u.%03u ms)\r\n",
               us, ms, us % 1000);

    // cleanup_platform();
    return 0;
}