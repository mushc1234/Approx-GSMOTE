// arrival_aggregator.sv
// =====================================================================
// Min-reducer across the W parallel slot cmp outputs.
//
// The slot's pipelined distance unit emits (cmp_valid, cmp_dist, cmp_tag)
// where cmp_tag carries {resident_idx, arrival_idx} (the slot's defensive
// packed-tag design). All W slots are in lock-step pipeline phase, so on
// any given cycle either all of them emit cmp_valid together (when the
// current arrival's pipeline output is emerging) or none do.
//
// The aggregator finds the (slot with the smallest cmp_dist) and emits
// its (cmp_dist, cmp_tag) on a registered output. The winning tag
// identifies both:
//   - the resident_idx that produced the closest distance, and
//   - the arrival_idx the comparison was for (lower half of the tag).
// The engine consumes this as: "when arrival_idx becomes the next loaded
// resident, its initial best_idx should be the upper half of this tag,
// and its initial best_dist is this dist." This is the load-init-best
// path that closes the boundary-effect gap.
//
// Architecture: combinational tree of 2-input min comparators, output
// registered (1 cycle latency). The tree handles invalid inputs by
// substituting MAX as their dist, so invalid slots always lose. The
// output is valid iff any input was valid.
//
// W is assumed power-of-2. Verilator will complain at elaboration if
// not, via the $clog2 vs literal-shift mismatch in the tree generate.
// =====================================================================

module arrival_aggregator #(
    parameter int W         = 8,
    parameter int ACC_WIDTH = 40,
    parameter int TAG_WIDTH = 32      // = IDX_WIDTH * 2 from the slot
) (
    input  logic                          clk,
    input  logic                          rst_n,

    // W parallel inputs, one per slot
    input  logic                          slot_cmp_valid [W],
    input  logic [ACC_WIDTH-1:0]          slot_cmp_dist  [W],
    input  logic [TAG_WIDTH-1:0]          slot_cmp_tag   [W],

    // Registered aggregated output (latency = 1 cycle)
    output logic                          out_valid,
    output logic [ACC_WIDTH-1:0]          out_min_dist,
    output logic [TAG_WIDTH-1:0]          out_min_tag
);

    localparam int TREE_DEPTH = $clog2(W);

    // -----------------------------------------------------------------
    // Mask invalid slots: substitute MAX dist so they lose comparisons.
    // Tag is don't-care when valid is low.
    // -----------------------------------------------------------------
    logic [ACC_WIDTH-1:0] masked_dist [W];
    logic [TAG_WIDTH-1:0] masked_tag  [W];

    always_comb begin
        for (int i = 0; i < W; i++) begin
            masked_dist[i] = slot_cmp_valid[i] ? slot_cmp_dist[i]
                                               : {ACC_WIDTH{1'b1}};
            masked_tag [i] = slot_cmp_tag[i];
        end
    end

    // -----------------------------------------------------------------
    // "any input valid" OR-reduction, then register the output.
    // -----------------------------------------------------------------
    logic any_valid_in;
    always_comb begin
        any_valid_in = 1'b0;
        for (int i = 0; i < W; i++) any_valid_in |= slot_cmp_valid[i];
    end

    // -----------------------------------------------------------------
    // Min-reducer tree, expressed as a single always_comb so the data
    // flow (level lvl -> level lvl+1) is unambiguously acyclic from the
    // tool's perspective. Splitting this across multiple always_comb
    // blocks via generate triggers spurious UNOPTFLAT warnings because
    // the lint's array dependency analysis is conservative.
    //
    // The 2-D array is over-allocated to [TREE_DEPTH+1][W]; only the
    // first (W >> lvl) entries of each row are written. Synthesis
    // optimises the unused entries away.
    // -----------------------------------------------------------------
    /* verilator lint_off UNUSED */
    logic [ACC_WIDTH-1:0] tree_dist [TREE_DEPTH+1][W];
    logic [TAG_WIDTH-1:0] tree_tag  [TREE_DEPTH+1][W];
    /* verilator lint_on UNUSED */

    logic [TREE_DEPTH+1:0] valid_pipe;

    always_ff @ (posedge clk) begin
        // Level 0: copy masked inputs
        for (int i = 0; i < W; i++) begin
            tree_dist[0][i] <= masked_dist[i];
            tree_tag [0][i] <= masked_tag [i];
        end
        valid_pipe[0] <= any_valid_in

        for (int lvl = 0;lvl < TREE_DEPTH ; lvl++) begin
            for (int idx = 0; idx < (W >> (lvl + 1)); idx++) begin
                if (tree_dist[lvl][2*idx] <= tree_dist[lvl][2*idx + 1]) begin
                    tree_dist[lvl+1][idx] <= tree_dist[lvl][2*idx];
                    tree_tag [lvl+1][idx] <= tree_tag [lvl][2*idx];
                end else begin
                    tree_dist[lvl+1][idx] <= tree_dist[lvl][2*idx + 1];
                    tree_tag [lvl+1][idx] <= tree_tag [lvl][2*idx + 1];
                end
            end
            valid_pipe[lvl+1] <= valid_pipe[lvl];
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            out_valid    <= 1'b0;
            out_min_dist <= '0;
            out_min_tag  <= '0;
        end else begin
            out_valid    <= any_valid_in[TREE_DEPTH];
            out_min_dist <= tree_dist[TREE_DEPTH][0];
            out_min_tag  <= tree_tag [TREE_DEPTH][0];
        end
    end

endmodule
