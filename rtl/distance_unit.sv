// distance_unit.sv
// =====================================================================
// Pipelined squared Euclidean distance between two D-dim vectors.
//
// This is the canonical shape of pipelined arithmetic on FPGAs:
//   - Each "stage" of work ends with a flip-flop, isolating its timing
//     path from the next stage. Synthesis sees short combinational paths
//     between stages and can hit a high clock target.
//   - Valid and any tag/metadata travel in a shift-register delay line
//     that runs alongside the datapath, so they emerge at the same cycle
//     as the data they describe.
//   - Throughput is one result per cycle after the pipeline fills.
//     Latency is the depth of the pipeline.
//
// Stages (one cycle each):
//   1.  diff[k] = a[k] - b[k]                  -> diff_q[D]
//   2.  sq[k]   = diff_q[k] * diff_q[k]        -> tree_q[0][D]
//   3+. adder tree, halving each level         -> tree_q[lvl][D>>lvl]
//
// Total latency = TREE_DEPTH + 2 cycles
//   D=8  -> 5 cycles
//   D=16 -> 6 cycles
//   D=32 -> 7 cycles
//
// Numeric reasoning is identical to the combinational version:
//   diff: IN_WIDTH+1 signed bits   sq: 2*IN_WIDTH unsigned bits
//   tree levels carry ACC_WIDTH unsigned bits (zero-extend on entry).
//
// Requires D to be a power of 2. The surrounding logic pads inputs if
// the dataset's actual dimensionality isn't a power of 2.
// =====================================================================

module distance_unit #(
    parameter int D         = 8,
    parameter int IN_WIDTH  = 16,
    parameter int ACC_WIDTH = 40,
    parameter int TAG_WIDTH = 32
) (
    input  logic                          clk,
    input  logic                          rst_n,

    input  logic                          in_valid,
    input  logic [D*IN_WIDTH-1:0]         in_a,
    input  logic [D*IN_WIDTH-1:0]         in_b,
    input  logic [TAG_WIDTH-1:0]          in_tag,

    output logic                          out_valid,
    output logic [ACC_WIDTH-1:0]          out_dist,
    output logic [TAG_WIDTH-1:0]          out_tag
);

    // Pipeline depth derived from D.  $clog2(8) = 3, $clog2(32) = 5.
    localparam int TREE_DEPTH = $clog2(D);
    localparam int LATENCY    = TREE_DEPTH + 2;

    // -----------------------------------------------------------------
    // Stage 1: D parallel subtractions, registered.
    // -----------------------------------------------------------------
    logic signed [IN_WIDTH:0] diff_q [D];

    always_ff @(posedge clk) begin
        for (int k = 0; k < D; k++) begin
            diff_q[k] <= $signed(in_a[(k+1)*IN_WIDTH-1 -: IN_WIDTH])
                       - $signed(in_b[(k+1)*IN_WIDTH-1 -: IN_WIDTH]);
        end
    end

    // -----------------------------------------------------------------
    // Stage 2: D parallel squarings, registered into the bottom of the
    // adder tree (level 0).  Zero-extend to ACC_WIDTH at this point so
    // subsequent additions are uniformly ACC_WIDTH wide.
    //
    // Stages 3..2+TREE_DEPTH: adder tree.  Level lvl reads from
    // tree_q[lvl] (which has D>>lvl active entries) and writes
    // tree_q[lvl+1] (D>>(lvl+1) entries) on the next clock edge.
    //
    // The 2-D array is over-allocated for simplicity: only D>>lvl
    // entries of each row are written. Synthesis optimises the unused
    // entries away. (Verilator may flag the unused entries -- if it
    // does, suppress with lint_off UNUSED at the declaration.)
    // -----------------------------------------------------------------
    logic [ACC_WIDTH-1:0] tree_q [TREE_DEPTH+1][D];

    always_ff @(posedge clk) begin
        for (int k = 0; k < D; k++) begin
            tree_q[0][k] <= ACC_WIDTH'(diff_q[k] * diff_q[k]);
        end
    end

    genvar lvl, idx;
    generate
        for (lvl = 0; lvl < TREE_DEPTH; lvl++) begin : tree_level
            for (idx = 0; idx < (D >> (lvl+1)); idx++) begin : tree_node
                always_ff @(posedge clk) begin
                    tree_q[lvl+1][idx] <= tree_q[lvl][2*idx]
                                        + tree_q[lvl][2*idx + 1];
                end
            end
        end
    endgenerate

    assign out_dist = tree_q[TREE_DEPTH][0];

    // -----------------------------------------------------------------
    // Valid and tag delay line. valid_pipe[i] holds in_valid from i+1
    // cycles ago; tag_pipe[i] is the same for in_tag. The outputs read
    // the LATENCY-1 slot, so they arrive aligned with out_dist.
    //
    // Note: we ONLY reset valid_pipe (correctness matters -- we don't
    // want spurious out_valid asserting after reset). We don't reset
    // tag_pipe because tag is meaningless when valid is low.
    // -----------------------------------------------------------------
    logic                  valid_pipe [LATENCY];
    logic [TAG_WIDTH-1:0]  tag_pipe   [LATENCY];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < LATENCY; i++) begin
                valid_pipe[i] <= 1'b0;
            end
        end else begin
            valid_pipe[0] <= in_valid;
            tag_pipe[0]   <= in_tag;
            for (int i = 1; i < LATENCY; i++) begin
                valid_pipe[i] <= valid_pipe[i-1];
                tag_pipe[i]   <= tag_pipe[i-1];
            end
        end
    end

    assign out_valid = valid_pipe[LATENCY-1];
    assign out_tag   = tag_pipe  [LATENCY-1];

endmodule