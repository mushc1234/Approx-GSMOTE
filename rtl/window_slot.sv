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
    parameter int D         = 4,
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

    // Aggregator inputs
    input  logic [IDX_WIDTH*2-1:0]            agg_best_tag,
    input  logic [ACC_WIDTH-1:0]              agg_best_dist,
    input  logic                              agg_best_valid,

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
    output logic [ACC_WIDTH-1:0]              cmp_dist,
    output logic                              cmp_valid,
    output logic [IDX_WIDTH*2-1:0]            cmp_tag
);

    logic signed [D*IN_WIDTH-1:0] resident_data_r;

    distance_unit #(
        .D(D),
        .IN_WIDTH(IN_WIDTH),
        .ACC_WIDTH(ACC_WIDTH),
        .TAG_WIDTH(IDX_WIDTH*2)
    ) dist_unit (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(arrival_valid && resident_valid && !load_valid),
        .in_a(resident_data_r),
        .in_b(arrival_data),
        .in_tag({resident_idx, arrival_idx}),
        .out_valid(cmp_valid),
        .out_tag(cmp_tag),
        .out_dist(cmp_dist)
    );

    logic [ACC_WIDTH-1:0] valid_cmp_dist;
    logic [ACC_WIDTH-1:0] valid_agg_dist;

    assign valid_cmp_dist = (cmp_valid && cmp_tag[IDX_WIDTH*2-1:IDX_WIDTH] == resident_idx) ? cmp_dist
                                                                                            : {ACC_WIDTH{1'b1}};
    assign valid_agg_dist = (agg_best_tag[IDX_WIDTH-1:0] == resident_idx && agg_best_valid) ? agg_best_dist
                                                                                            : {ACC_WIDTH{1'b1}};

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            best_idx  <= '0;
            best_dist <= {ACC_WIDTH{1'b1}};  // max unsigned value as sentinel
            resident_valid <= 0;
            resident_idx   <= '0;
            resident_data_r <= '0;
        end
        else begin
            if (load_valid) begin
                resident_valid  <= 1;
                resident_idx    <= load_idx;
                resident_data_r <= load_data;
                best_dist       <= {ACC_WIDTH{1'b1}};
                best_idx        <= '0; 
            end
            else if (resident_valid) begin
                if (valid_cmp_dist < best_dist || valid_agg_dist < best_dist) begin
                    best_dist <= (valid_cmp_dist < valid_agg_dist)? valid_cmp_dist : valid_agg_dist;
                    best_idx  <= (valid_cmp_dist < valid_agg_dist)? cmp_tag[IDX_WIDTH-1:0] : agg_best_tag[IDX_WIDTH*2-1:IDX_WIDTH];
                end
            end
        end
    end


endmodule
