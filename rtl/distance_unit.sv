// distance_unit.sv
// =====================================================================
// Squared Euclidean distance between two D-dim vectors.
//
//   out_dist = sum over k in 0..D-1 of (a[k] - b[k])^2
//
// Currently single-cycle combinational. It will be pipelined later by
// inserting registers between adder-tree levels for timing closure on
// the Artix-7 -- the interface is already shaped for that, with valid
// and tag passing through alongside data. When pipelined, out_valid
// becomes a delayed version of in_valid and out_tag tells the consumer
// which (arrival, slot) pair the emerging distance belongs to.
//
// Numeric reasoning:
//   diff = a[k] - b[k]:  IN_WIDTH-bit signed minus IN_WIDTH-bit signed,
//                        result needs IN_WIDTH+1 signed bits to avoid
//                        overflow on full-range inputs.
//   sq   = diff * diff:  Fits in 2*IN_WIDTH bits unsigned (diff^2 max
//                        is (2^IN_WIDTH - 1)^2 < 2^(2*IN_WIDTH)).
//   sum  = sum of D such squares: needs 2*IN_WIDTH + ceil(log2(D)) bits.
//                                 For D=32, IN_WIDTH=16: 37 bits.
//                                 ACC_WIDTH=40 gives 3 bits of margin.
// =====================================================================

module distance_unit #(
    parameter int D         = 8,
    parameter int IN_WIDTH  = 16,
    parameter int ACC_WIDTH = 40,
    parameter int TAG_WIDTH = 32
) (
    input  logic                          clk,
    input  logic                          rst_n,

    // Input pair
    input  logic                          in_valid,
    input  logic [D*IN_WIDTH-1:0]         in_a,
    input  logic [D*IN_WIDTH-1:0]         in_b,
    input  logic [TAG_WIDTH-1:0]          in_tag,

    // Output distance (combinational, latency = 0)
    output logic                          out_valid,
    output logic [ACC_WIDTH-1:0]          out_dist,
    output logic [TAG_WIDTH-1:0]          out_tag
);

    always_comb begin
        out_dist = '0;
        for (int k = 0; k < D; k++) begin
            automatic logic signed [IN_WIDTH:0]     diff;
            automatic logic        [2*IN_WIDTH-1:0] sq;
            diff = $signed(in_a[(k+1)*IN_WIDTH-1 -: IN_WIDTH])
                 - $signed(in_b[(k+1)*IN_WIDTH-1 -: IN_WIDTH]);
            sq   = diff * diff;
            out_dist = out_dist + ACC_WIDTH'(sq);
        end
    end

    // Combinational pass-through for now. When pipelining, replace these
    // with shift-register chains of the appropriate depth (matching the
    // adder-tree pipeline depth so tag/valid stay aligned with the data).
    assign out_valid = in_valid;
    assign out_tag   = in_tag;

    // clk and rst_n are unused in the combinational version but kept on
    // the port so the pipelined replacement is a drop-in. Suppress the
    // UNUSED lint until then.
    /* verilator lint_off UNUSED */
    wire _unused_clk_rst = &{1'b0, clk, rst_n};
    /* verilator lint_on UNUSED */

endmodule
