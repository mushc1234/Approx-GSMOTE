// window_slot.sv
// =====================================================================
// One slot of the sliding window in the signature-window NN engine.
//
// A slot holds one "resident" point and the running-best for that point.
// When a new arrival is broadcast across the window, this slot:
//   (a) computes squared Euclidean distance from its resident to the arrival,
//       outputting it combinationally on `cmp_dist` so the arrival-side
//       aggregator (elsewhere) can find the minimum across all W slots,
//   (b) updates its resident's running-best register if the arrival is
//       closer than anything seen so far. This is the dual-role update --
//       every comparison updates BOTH parties, not just the arrival.
//
// Numeric format: see FIXED_POINT.md from the golden generator. In brief,
//   data is signed IN_WIDTH-bit; distance is unsigned ACC_WIDTH-bit;
//   indices are unsigned IDX_WIDTH-bit. Data ports are packed
//   (dim k occupies bits [(k+1)*IN_WIDTH-1 -: IN_WIDTH] of the vector).
//
// Cycle behaviour the testbench expects:
//   posedge clk:
//     if !rst_n:        clear resident_valid; set best_dist to sentinel (max)
//     elif load_valid:  latch load_data + load_idx as new resident;
//                       reset best_dist to sentinel, best_idx don't care
//     elif arrival_valid && resident_valid:
//                       if cmp_dist < best_dist:
//                           best_dist <= cmp_dist;  best_idx <= arrival_idx
// Combinational:
//   cmp_dist = sum over k of (signed(resident[k]) - signed(arrival[k]))^2
//   when (resident_valid && arrival_valid); otherwise don't-care.
// =====================================================================

module window_slot #(
    parameter int D         = 2,
    parameter int IN_WIDTH  = 16,
    parameter int ACC_WIDTH = 40,
    parameter int IDX_WIDTH = 16
) (
    input  logic                              clk,
    input  logic                              rst_n,

    // Resident-load port
    input  logic                              load_valid,
    input  logic [IDX_WIDTH-1:0]              load_idx,
    input  logic [D*IN_WIDTH-1:0]             load_data,

    // Arrival broadcast port
    input  logic                              arrival_valid,
    input  logic [IDX_WIDTH-1:0]              arrival_idx,
    input  logic [D*IN_WIDTH-1:0]             arrival_data,

    // Slot status outputs (registered)
    output logic                              resident_valid,
    output logic [IDX_WIDTH-1:0]              resident_idx,
    output logic [IDX_WIDTH-1:0]              best_idx,
    output logic [ACC_WIDTH-1:0]              best_dist,

    // Distance to current arrival (combinational, valid when arrival_valid)
    output logic [ACC_WIDTH-1:0]              cmp_dist
);

    // ===================================================================
    // STUB IMPLEMENTATION
    //
    // All outputs are driven to zero so the design compiles. Every check
    // in the testbench should FAIL on the first build -- that's how you
    // know the toolchain works. Replace this stub piece by piece:
    //
    //   1. Resident latch (gives you `resident_valid` and `resident_idx`)
    //   2. Combinational distance unit (gives you `cmp_dist`)
    //   3. Running-best update (gives you `best_idx` and `best_dist`)
    //
    // After each piece, re-run `make` -- progressively more checks pass.
    // ===================================================================

    assign resident_valid = 1'b0;

    always @(posedge clk) begin
        if (rst_n) begin
            resident_valid <= 0;
        end
        else begin
            if(load_valid)
                resident_valid <= 1;
        end
    end

    assign resident_idx   = '0;
    assign best_idx       = '0;
    assign best_dist      = '0;
    assign cmp_dist       = '0;

endmodule
