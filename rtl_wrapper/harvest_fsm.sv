// harvest_fsm.sv
// =====================================================================
// Drains harvest tuples from the engine, and merges each into the
// result BRAM with a compare-and-write step. This is what realises
// multi-pass aggregation: across passes, the same resident_idx may be
// harvested multiple times; we keep the smallest best_dist.
//
// Protocol with the engine:
//   - When idle (S_ACCEPT), out_ready is asserted.
//   - When a tuple is accepted (out_valid && out_ready), we latch it,
//     drop out_ready, and issue a BRAM read at the tuple's resident_idx.
//   - The next cycle (S_COMPARE), the BRAM read data is available.
//     We compare and, if the new tuple is better, drive a full-write
//     back to the BRAM. Then we return to S_ACCEPT.
//
// Performance counters:
//   - harvest_count_pulse: pulses once per accepted tuple
//   - improvement_count_pulse: pulses once per accepted tuple that
//     actually improved the BRAM (new entry or strictly smaller dist)
//
// Engine sees out_ready=1 on alternate cycles in the steady state.
// =====================================================================

module harvest_fsm #(
    parameter int N_MAX_LOG2      = 14,
    parameter int IDX_WIDTH       = 16,
    parameter int ACC_WIDTH       = 40,
    parameter int ACC_STORE_WIDTH = 48
) (
    input  logic                       clk,
    input  logic                       rst_n,

    input  logic                       enable,            // gates out_ready outside of a run

    // -------- Engine harvest stream --------
    input  logic                       eng_out_valid,
    output logic                       eng_out_ready,
    input  logic [IDX_WIDTH-1:0]       eng_out_resident_idx,
    input  logic [IDX_WIDTH-1:0]       eng_out_best_idx,
    input  logic [ACC_WIDTH-1:0]       eng_out_best_dist,

    // -------- Result BRAM port A --------
    output logic [N_MAX_LOG2-1:0]      bram_rd_addr,
    input  logic                       bram_rd_valid_bit,
    /* verilator lint_off UNUSED */
    input  logic [IDX_WIDTH-1:0]       bram_rd_resident_idx,
    input  logic [IDX_WIDTH-1:0]       bram_rd_best_idx,
    /* verilator lint_on UNUSED */
    input  logic [ACC_STORE_WIDTH-1:0] bram_rd_best_dist,

    output logic                       bram_wr_full_en,
    output logic [N_MAX_LOG2-1:0]      bram_wr_addr,
    output logic                       bram_wr_valid_bit,
    output logic [IDX_WIDTH-1:0]       bram_wr_resident_idx,
    output logic [IDX_WIDTH-1:0]       bram_wr_best_idx,
    output logic [ACC_STORE_WIDTH-1:0] bram_wr_best_dist,

    // -------- Counter pulses --------
    output logic                       harvest_count_pulse,
    output logic                       improvement_count_pulse
);

    typedef enum logic [0:0] {
        S_ACCEPT,
        S_COMPARE
    } state_t;

    state_t                       state_r;
    logic [IDX_WIDTH-1:0]         lat_resident_r;
    logic [IDX_WIDTH-1:0]         lat_best_r;
    logic [ACC_WIDTH-1:0]         lat_dist_r;

    // resident_idx may be wider than N_MAX_LOG2 in principle; truncate.
    logic [N_MAX_LOG2-1:0]        resident_addr;
    assign resident_addr = lat_resident_r[N_MAX_LOG2-1:0];

    // -----------------------------------------------------------------
    // Compare: write if BRAM entry not valid, OR new dist < stored dist
    // -----------------------------------------------------------------
    logic [ACC_STORE_WIDTH-1:0]   lat_dist_ext;
    assign lat_dist_ext = { {(ACC_STORE_WIDTH-ACC_WIDTH){1'b0}}, lat_dist_r };

    logic                         is_improvement;
    assign is_improvement = !bram_rd_valid_bit
                          || (lat_dist_ext < bram_rd_best_dist);

    // -----------------------------------------------------------------
    // State / datapath
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state_r        <= S_ACCEPT;
            lat_resident_r <= '0;
            lat_best_r     <= '0;
            lat_dist_r     <= '0;
        end else begin
            unique case (state_r)
                S_ACCEPT: begin
                    if (enable && eng_out_valid) begin
                        lat_resident_r <= eng_out_resident_idx;
                        lat_best_r     <= eng_out_best_idx;
                        lat_dist_r     <= eng_out_best_dist;
                        state_r        <= S_COMPARE;
                    end
                end
                S_COMPARE: begin
                    state_r <= S_ACCEPT;
                end
                default: state_r <= S_ACCEPT;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // Outputs
    // -----------------------------------------------------------------
    always_comb begin
        eng_out_ready = enable && (state_r == S_ACCEPT);

        // BRAM read address comes from the engine signal when we accept,
        // so that the very next cycle (when we're in S_COMPARE) the read
        // data reflects the latched tuple's resident_idx. The read addr
        // is registered into the BRAM, so we drive the *unregistered*
        // input value on the cycle we accept.
        if (state_r == S_ACCEPT && eng_out_valid && enable) begin
            bram_rd_addr = eng_out_resident_idx[N_MAX_LOG2-1:0];
        end else begin
            bram_rd_addr = resident_addr;
        end

        // BRAM write fires in S_COMPARE iff improving
        bram_wr_full_en      = (state_r == S_COMPARE) && is_improvement;
        bram_wr_addr         = resident_addr;
        bram_wr_valid_bit    = 1'b1;
        bram_wr_resident_idx = lat_resident_r;
        bram_wr_best_idx     = lat_best_r;
        bram_wr_best_dist    = lat_dist_ext;

        harvest_count_pulse     = (state_r == S_COMPARE);
        improvement_count_pulse = (state_r == S_COMPARE) && is_improvement;
    end

endmodule
