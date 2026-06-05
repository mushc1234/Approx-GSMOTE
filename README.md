# Window Slot — RTL bring-up

First testbench in the signature-window NN accelerator. A *window slot* holds one
"resident" point and that point's running-best. When new arrivals are broadcast
across the window, the slot (a) outputs its distance to the arrival for the
arrival-side aggregator and (b) updates its own resident's running-best when the
arrival is closer than anything seen before. This is the dual-role update at the
heart of the engine.

## Setup

Install Verilator (4.200 or newer recommended):

    Ubuntu / WSL:    sudo apt install verilator
    macOS:           brew install verilator
    From source:     https://verilator.org

Optional, for waveform viewing:

    Ubuntu / WSL:    sudo apt install gtkwave
    macOS:           brew install --cask gtkwave

## Run

    make            # build + run the testbench
    make wave       # also open the waveform in GTKWave
    make clean      # remove build artefacts

On the very first build the test FAILS — that's expected. The DUT in
`rtl/window_slot.sv` is a stub that drives every output to zero. Each
failing check tells you exactly which signal needs to come alive.

## What to implement

Open `rtl/window_slot.sv`. The header comment describes the expected
cycle behaviour. Suggested order of implementation (each step makes more
checks pass without breaking earlier ones):

  1. **Resident latch.** On `posedge clk` with `load_valid` high, latch
     `load_data` and `load_idx` into resident registers, set
     `resident_valid` high, and initialise `best_dist` to the maximum
     representable value (sentinel). On `!rst_n`, clear `resident_valid`.

     → passes: `resident_valid` and `resident_idx` checks.

  2. **Combinational distance.** Compute `cmp_dist` as the sum over
     `k = 0..D-1` of `(signed(resident_data[k]) - signed(arrival_data[k]))^2`.
     Extract per-dimension fields using
     `arrival_data[(k+1)*IN_WIDTH-1 -: IN_WIDTH]`.

     → passes: `cmp_dist` checks.

  3. **Running-best update.** On `posedge clk` with `arrival_valid` and
     `resident_valid` both high, if `cmp_dist < best_dist`, latch
     `best_idx <= arrival_idx` and `best_dist <= cmp_dist`.

     → passes: `best_idx` and `best_dist` checks.

## After this passes

Subsequent testbenches (not yet provided in this starter):

- **`tb_distance_unit`** — distance unit standalone, driven exhaustively
  by `dist_pairs_all.csv` from the golden generator.
- **`tb_window_engine`** — W slots in parallel with a broadcast bus and
  arrival-side aggregator. Driven by `window_trace.csv` and the directed
  `directed_dual_role.csv` test.
- **`tb_full_pipeline`** — end-to-end against `final_nn.csv`.

The pattern in `tb_window_slot.cpp` (Verilator boilerplate, `tick()`,
`check()` helpers) carries over directly; clone it and adapt.

## A few things worth knowing if Verilator is new

- Verilator is strict about SystemVerilog. Things Vivado happily ignores
  (sloppy sensitivity lists, implicit nets) Verilator flags as errors.
  This is good; the errors are the language's way of pointing at real bugs.
- Waveforms are dumped to `waves.vcd`. Open in GTKWave; the
  `tb_window_slot` module hierarchy is at the top, the DUT inside.
- The Makefile rebuilds whenever `rtl/window_slot.sv` or
  `tb/tb_window_slot.cpp` change. `make clean` if anything looks weird.
