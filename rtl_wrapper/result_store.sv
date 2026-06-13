// result_store.sv
// =====================================================================
// Composite of result_bram + clear_fsm + harvest_fsm. Port A of the
// BRAM is arbitrated between the clear FSM (which only writes the
// valid bit) and the harvest FSM (which does full-entry compare and
// write). At runtime only one of them is active at a time -- they're
// driven by different phases of the top-level run FSM -- but we still
// gate the arbitration explicitly here so unit tests can drive either
// safely.
// =====================================================================

module result_store #(
    parameter int N_MAX_LOG2      = 14,
    parameter int IDX_WIDTH       = 16,
    parameter int ACC_WIDTH       = 40,
    parameter int ACC_STORE_WIDTH = 48
) (
    input  logic                       clk,
    input  logic                       rst_n,

    // -------- Clear control --------
    input  logic                       clear_start,
    output logic                       clear_busy,
    output logic                       clear_done,

    // -------- Harvest enable + engine port --------
    input  logic                       harvest_enable,
    input  logic                       eng_out_valid,
    output logic                       eng_out_ready,
    input  logic [IDX_WIDTH-1:0]       eng_out_resident_idx,
    input  logic [IDX_WIDTH-1:0]       eng_out_best_idx,
    input  logic [ACC_WIDTH-1:0]       eng_out_best_dist,

    // -------- Counter pulses --------
    output logic                       harvest_count_pulse,
    output logic                       improvement_count_pulse,

    // -------- CPU/MMIO read port --------
    input  logic                       b_rd_en,
    input  logic [N_MAX_LOG2-1:0]      b_rd_entry,
    input  logic [1:0]                 b_rd_word,
    output logic [31:0]                b_rd_data
);

    // -------- Clear-side BRAM signals --------
    logic                       clr_wr_valid_en;
    logic [N_MAX_LOG2-1:0]      clr_wr_addr;
    logic                       clr_wr_valid_bit;

    clear_fsm #(
        .N_MAX_LOG2(N_MAX_LOG2)
    ) u_clear (
        .clk               (clk),
        .rst_n             (rst_n),
        .start             (clear_start),
        .busy              (clear_busy),
        .done              (clear_done),
        .bram_wr_valid_en  (clr_wr_valid_en),
        .bram_wr_addr      (clr_wr_addr),
        .bram_wr_valid_bit (clr_wr_valid_bit)
    );

    // -------- Harvest-side BRAM signals --------
    logic [N_MAX_LOG2-1:0]      hv_rd_addr;
    logic                       hv_rd_valid_bit;
    logic [IDX_WIDTH-1:0]       hv_rd_resident_idx;
    logic [IDX_WIDTH-1:0]       hv_rd_best_idx;
    logic [ACC_STORE_WIDTH-1:0] hv_rd_best_dist;

    logic                       hv_wr_full_en;
    logic [N_MAX_LOG2-1:0]      hv_wr_addr;
    logic                       hv_wr_valid_bit;
    logic [IDX_WIDTH-1:0]       hv_wr_resident_idx;
    logic [IDX_WIDTH-1:0]       hv_wr_best_idx;
    logic [ACC_STORE_WIDTH-1:0] hv_wr_best_dist;

    harvest_fsm #(
        .N_MAX_LOG2     (N_MAX_LOG2),
        .IDX_WIDTH      (IDX_WIDTH),
        .ACC_WIDTH      (ACC_WIDTH),
        .ACC_STORE_WIDTH(ACC_STORE_WIDTH)
    ) u_harvest (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .enable                 (harvest_enable),
        .eng_out_valid          (eng_out_valid),
        .eng_out_ready          (eng_out_ready),
        .eng_out_resident_idx   (eng_out_resident_idx),
        .eng_out_best_idx       (eng_out_best_idx),
        .eng_out_best_dist      (eng_out_best_dist),
        .bram_rd_addr           (hv_rd_addr),
        .bram_rd_valid_bit      (hv_rd_valid_bit),
        .bram_rd_resident_idx   (hv_rd_resident_idx),
        .bram_rd_best_idx       (hv_rd_best_idx),
        .bram_rd_best_dist      (hv_rd_best_dist),
        .bram_wr_full_en        (hv_wr_full_en),
        .bram_wr_addr           (hv_wr_addr),
        .bram_wr_valid_bit      (hv_wr_valid_bit),
        .bram_wr_resident_idx   (hv_wr_resident_idx),
        .bram_wr_best_idx       (hv_wr_best_idx),
        .bram_wr_best_dist      (hv_wr_best_dist),
        .harvest_count_pulse    (harvest_count_pulse),
        .improvement_count_pulse(improvement_count_pulse)
    );

    // -------- BRAM port A arbitration --------
    // Clear takes precedence; harvest only runs when clear is idle.
    // In the top-level run FSM these are sequenced so neither talks
    // over the other anyway, but make the priority explicit.
    logic                       muxed_wr_full_en;
    logic                       muxed_wr_valid_en;
    logic [N_MAX_LOG2-1:0]      muxed_wr_addr;
    logic                       muxed_wr_valid_bit;
    logic [IDX_WIDTH-1:0]       muxed_wr_resident_idx;
    logic [IDX_WIDTH-1:0]       muxed_wr_best_idx;
    logic [ACC_STORE_WIDTH-1:0] muxed_wr_best_dist;

    always_comb begin
        if (clr_wr_valid_en) begin
            muxed_wr_full_en      = 1'b0;
            muxed_wr_valid_en     = 1'b1;
            muxed_wr_addr         = clr_wr_addr;
            muxed_wr_valid_bit    = clr_wr_valid_bit;
            muxed_wr_resident_idx = '0;
            muxed_wr_best_idx     = '0;
            muxed_wr_best_dist    = '0;
        end else begin
            muxed_wr_full_en      = hv_wr_full_en;
            muxed_wr_valid_en     = 1'b0;
            muxed_wr_addr         = hv_wr_addr;
            muxed_wr_valid_bit    = hv_wr_valid_bit;
            muxed_wr_resident_idx = hv_wr_resident_idx;
            muxed_wr_best_idx     = hv_wr_best_idx;
            muxed_wr_best_dist    = hv_wr_best_dist;
        end
    end

    result_bram #(
        .N_MAX_LOG2     (N_MAX_LOG2),
        .IDX_WIDTH      (IDX_WIDTH),
        .ACC_STORE_WIDTH(ACC_STORE_WIDTH)
    ) u_bram (
        .clk                (clk),
        .rst_n              (rst_n),
        .a_rd_addr          (hv_rd_addr),
        .a_rd_valid_bit     (hv_rd_valid_bit),
        .a_rd_resident_idx  (hv_rd_resident_idx),
        .a_rd_best_idx      (hv_rd_best_idx),
        .a_rd_best_dist     (hv_rd_best_dist),
        .a_wr_full_en       (muxed_wr_full_en),
        .a_wr_valid_en      (muxed_wr_valid_en),
        .a_wr_addr          (muxed_wr_addr),
        .a_wr_valid_bit     (muxed_wr_valid_bit),
        .a_wr_resident_idx  (muxed_wr_resident_idx),
        .a_wr_best_idx      (muxed_wr_best_idx),
        .a_wr_best_dist     (muxed_wr_best_dist),
        .b_rd_en            (b_rd_en),
        .b_rd_entry         (b_rd_entry),
        .b_rd_word          (b_rd_word),
        .b_rd_data          (b_rd_data)
    );

endmodule
