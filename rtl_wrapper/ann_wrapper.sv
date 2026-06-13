// gsmote_wrapper.sv
// =====================================================================
// Top-level wrapper around the window_engine.
//
// External interfaces:
//   * AXI4-Lite slave (S_AXI_*) for CPU control/status and result MMIO
//   * AXI4 full master read (M_AXI_*) for fetching point records from DDR
//
// Internal subsystems:
//   * axi_lite_regs     -- register file + BRAM-read window
//   * run_fsm           -- top-level sequencer
//   * clear_fsm         -- result BRAM clear (inside result_store)
//   * harvest_fsm       -- compare-and-write into result BRAM
//   * result_bram       -- per-resident running-best store
//   * axi_master_read   -- one INCR burst per record
//   * record_splitter   -- beats -> {id, data} bundles
//   * feeder_fsm        -- drives engine in_arrival_*
//   * window_engine     -- the accelerator core (unchanged)
// =====================================================================

module ann_wrapper #(
    parameter int W              = 8,
    parameter int D              = 8,
    parameter int LANES          = 4,
    parameter int IN_WIDTH       = 16,
    parameter int ACC_WIDTH      = 40,
    parameter int IDX_WIDTH      = 16,
    parameter int EVICT_INTERVAL = 1,

    parameter int N_MAX_LOG2     = 14,
    parameter int ACC_STORE_WIDTH = 48,

    parameter int AXI_LITE_ADDR_W = 18,    // covers 0x0000..0x3FFFF (256K BRAM window)
    parameter int AXI_ADDR_W      = 32,
    parameter int AXI_DATA_W      = 32
) (
    input  logic                          clk,
    input  logic                          rst_n,

    // ===== AXI4-Lite slave (control / status / result MMIO) =====
    input  logic [AXI_LITE_ADDR_W-1:0]    s_axi_awaddr,
    input  logic                          s_axi_awvalid,
    output logic                          s_axi_awready,
    input  logic [31:0]                   s_axi_wdata,
    input  logic [3:0]                    s_axi_wstrb,
    input  logic                          s_axi_wvalid,
    output logic                          s_axi_wready,
    output logic [1:0]                    s_axi_bresp,
    output logic                          s_axi_bvalid,
    input  logic                          s_axi_bready,
    input  logic [AXI_LITE_ADDR_W-1:0]    s_axi_araddr,
    input  logic                          s_axi_arvalid,
    output logic                          s_axi_arready,
    output logic [31:0]                   s_axi_rdata,
    output logic [1:0]                    s_axi_rresp,
    output logic                          s_axi_rvalid,
    input  logic                          s_axi_rready,

    // ===== AXI4 master read (DDR points fetch) =====
    output logic                          m_axi_arvalid,
    input  logic                          m_axi_arready,
    output logic [AXI_ADDR_W-1:0]         m_axi_araddr,
    output logic [7:0]                    m_axi_arlen,
    output logic [2:0]                    m_axi_arsize,
    output logic [1:0]                    m_axi_arburst,

    input  logic                          m_axi_rvalid,
    output logic                          m_axi_rready,
    input  logic [AXI_DATA_W-1:0]         m_axi_rdata,
    input  logic                          m_axi_rlast,
    input  logic [1:0]                    m_axi_rresp
);

    // Derived: AXI beats per record on the read bus.
    // Record layout = {2B id, 2B pad, D*IN_WIDTH/8 B data}.
    // Bytes = 4 + D*IN_WIDTH/8 ; beats = ceil(bytes / (AXI_DATA_W/8)).
    localparam  RECORD_BYTES   = 4 + (D * IN_WIDTH) / 8;
    localparam  BEATS_PER_REC  = (RECORD_BYTES + (AXI_DATA_W/8) - 1) / (AXI_DATA_W/8);

    // -----------------------------------------------------------------
    // Control plane signals
    // -----------------------------------------------------------------
    logic        ctrl_start_pulse;
    logic        ctrl_clear_pulse;
    logic        ctrl_abort_pulse;
    logic        status_done_clear_pulse;

    logic        status_idle, status_running, status_done;
    logic        status_clear_busy, status_error;

    logic [31:0] cfg_n_points;
    logic [31:0] cfg_points_ptr;
    logic [31:0] cfg_pass_id;
    /* verilator lint_off UNUSED */
    logic [31:0] cfg_pass_id_unused;
    assign cfg_pass_id_unused = cfg_pass_id;
    /* verilator lint_on UNUSED */

    logic [63:0] stat_cycle_count;
    logic [31:0] stat_harvest_count;
    logic [31:0] stat_improvement_count;

    // BRAM port B (MMIO read)
    logic                   bram_b_rd_en;
    logic [N_MAX_LOG2-1:0]  bram_b_rd_entry;
    logic [1:0]             bram_b_rd_word;
    logic [31:0]            bram_b_rd_data;

    axi_lite_regs #(
        .N_MAX_LOG2    (N_MAX_LOG2),
        .AXI_ADDR_WIDTH(AXI_LITE_ADDR_W),
        .AXI_DATA_WIDTH(32)
    ) u_regs (
        .clk                    (clk),
        .rst_n                  (rst_n),

        .s_axi_awaddr           (s_axi_awaddr),
        .s_axi_awvalid          (s_axi_awvalid),
        .s_axi_awready          (s_axi_awready),
        .s_axi_wdata            (s_axi_wdata),
        .s_axi_wstrb            (s_axi_wstrb),
        .s_axi_wvalid           (s_axi_wvalid),
        .s_axi_wready           (s_axi_wready),
        .s_axi_bresp            (s_axi_bresp),
        .s_axi_bvalid           (s_axi_bvalid),
        .s_axi_bready           (s_axi_bready),
        .s_axi_araddr           (s_axi_araddr),
        .s_axi_arvalid          (s_axi_arvalid),
        .s_axi_arready          (s_axi_arready),
        .s_axi_rdata            (s_axi_rdata),
        .s_axi_rresp            (s_axi_rresp),
        .s_axi_rvalid           (s_axi_rvalid),
        .s_axi_rready           (s_axi_rready),

        .ctrl_start_pulse       (ctrl_start_pulse),
        .ctrl_clear_pulse       (ctrl_clear_pulse),
        .ctrl_abort_pulse       (ctrl_abort_pulse),
        .status_done_clear_pulse(status_done_clear_pulse),

        .status_idle            (status_idle),
        .status_running         (status_running),
        .status_done            (status_done),
        .status_clear_busy      (status_clear_busy),
        .status_error           (status_error),

        .cfg_n_points           (cfg_n_points),
        .cfg_points_ptr         (cfg_points_ptr),
        .cfg_pass_id            (cfg_pass_id),

        .stat_cycle_count       (stat_cycle_count),
        .stat_harvest_count     (stat_harvest_count),
        .stat_improvement_count (stat_improvement_count),

        .bram_b_rd_en           (bram_b_rd_en),
        .bram_b_rd_entry        (bram_b_rd_entry),
        .bram_b_rd_word         (bram_b_rd_word),
        .bram_b_rd_data         (bram_b_rd_data)
    );

    // -----------------------------------------------------------------
    // Result store (BRAM + clear FSM + harvest FSM)
    // -----------------------------------------------------------------
    logic        clear_start_w;
    logic        clear_busy_w;
    logic        clear_done_w;

    logic        harvest_enable_w;
    logic        eng_out_valid;
    logic        eng_out_ready;
    logic [IDX_WIDTH-1:0] eng_out_resident_idx;
    logic [IDX_WIDTH-1:0] eng_out_best_idx;
    logic [ACC_WIDTH-1:0] eng_out_best_dist;

    logic        harvest_count_pulse;
    logic        improvement_count_pulse;

    // Counters
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            stat_harvest_count     <= '0;
            stat_improvement_count <= '0;
        end else if (ctrl_start_pulse) begin
            stat_harvest_count     <= '0;
            stat_improvement_count <= '0;
        end else begin
            if (harvest_count_pulse)
                stat_harvest_count <= stat_harvest_count + 1'b1;
            if (improvement_count_pulse)
                stat_improvement_count <= stat_improvement_count + 1'b1;
        end
    end

    result_store #(
        .N_MAX_LOG2     (N_MAX_LOG2),
        .IDX_WIDTH      (IDX_WIDTH),
        .ACC_WIDTH      (ACC_WIDTH),
        .ACC_STORE_WIDTH(ACC_STORE_WIDTH)
    ) u_result_store (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .clear_start            (clear_start_w),
        .clear_busy             (clear_busy_w),
        .clear_done             (clear_done_w),
        .harvest_enable         (harvest_enable_w),
        .eng_out_valid          (eng_out_valid),
        .eng_out_ready          (eng_out_ready),
        .eng_out_resident_idx   (eng_out_resident_idx),
        .eng_out_best_idx       (eng_out_best_idx),
        .eng_out_best_dist      (eng_out_best_dist),
        .harvest_count_pulse    (harvest_count_pulse),
        .improvement_count_pulse(improvement_count_pulse),
        .b_rd_en                (bram_b_rd_en),
        .b_rd_entry             (bram_b_rd_entry),
        .b_rd_word              (bram_b_rd_word),
        .b_rd_data              (bram_b_rd_data)
    );

    // -----------------------------------------------------------------
    // AXI master read + splitter + feeder
    // -----------------------------------------------------------------
    logic        axi_rd_start;
    logic        axi_rd_busy;
    logic        axi_rd_done;
    logic        feeder_enable;
    logic        feeder_reset_count;
    logic [31:0] records_pushed;

    logic                       beat_valid;
    logic                       beat_ready;
    logic [AXI_DATA_W-1:0]      beat_data;
    logic                       beat_last;

    axi_master_read #(
        .AXI_ADDR_WIDTH (AXI_ADDR_W),
        .AXI_DATA_WIDTH (AXI_DATA_W),
        .AXI_ID_WIDTH   (1),
        .MAX_BEATS_PER_REC_LOG2(5)
    ) u_axi_rd (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (axi_rd_start),
        .base_addr     (cfg_points_ptr),
        .n_records     (cfg_n_points),
        .stride_bytes  (32'(RECORD_BYTES)),
        .beats_per_rec (6'(BEATS_PER_REC)),
        .busy          (axi_rd_busy),
        .done          (axi_rd_done),
        /* verilator lint_off PINCONNECTEMPTY */
        .m_axi_arid    (),
        /* verilator lint_on PINCONNECTEMPTY */
        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arlen   (m_axi_arlen),
        .m_axi_arsize  (m_axi_arsize),
        .m_axi_arburst (m_axi_arburst),
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_rid     (1'b0),
        .m_axi_rresp   (m_axi_rresp),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rlast   (m_axi_rlast),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready),
        .out_valid     (beat_valid),
        .out_ready     (beat_ready),
        .out_data      (beat_data),
        .out_last      (beat_last)
    );

    logic                       rec_valid;
    logic                       rec_ready;
    logic [IDX_WIDTH-1:0]       rec_id;
    logic [D*IN_WIDTH-1:0]      rec_data;

    record_splitter #(
        .D             (D),
        .IN_WIDTH      (IN_WIDTH),
        .IDX_WIDTH     (IDX_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_W)
    ) u_splitter (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (beat_valid),
        .in_ready (beat_ready),
        .in_data  (beat_data),
        .in_last  (beat_last),
        .out_valid(rec_valid),
        .out_ready(rec_ready),
        .out_id   (rec_id),
        .out_data (rec_data)
    );

    logic                       eng_in_valid;
    logic                       eng_in_ready;
    logic [IDX_WIDTH-1:0]       eng_in_arrival_idx;
    logic [D*IN_WIDTH-1:0]      eng_in_arrival_data;

    feeder_fsm #(
        .D        (D),
        .IN_WIDTH (IN_WIDTH),
        .IDX_WIDTH(IDX_WIDTH)
    ) u_feeder (
        .clk                (clk),
        .rst_n              (rst_n),
        .enable             (feeder_enable),
        .reset_count        (feeder_reset_count),
        .rec_valid          (rec_valid),
        .rec_ready          (rec_ready),
        .rec_id             (rec_id),
        .rec_data           (rec_data),
        .eng_in_valid       (eng_in_valid),
        .eng_in_ready       (eng_in_ready),
        .eng_in_arrival_idx (eng_in_arrival_idx),
        .eng_in_arrival_data(eng_in_arrival_data),
        .records_pushed     (records_pushed)
    );

    // -----------------------------------------------------------------
    // Engine
    // -----------------------------------------------------------------
    logic eng_flush;
    logic eng_flush_done;

    window_engine #(
        .W             (W), 
        .D             (D),
        .LANES         (LANES),
        .IN_WIDTH      (IN_WIDTH),
        .ACC_WIDTH     (ACC_WIDTH),
        .IDX_WIDTH     (IDX_WIDTH),
        .EVICT_INTERVAL(EVICT_INTERVAL)
    ) u_engine (
        .clk             (clk),
        .rst_n           (rst_n),
        .in_valid        (eng_in_valid),
        .in_ready        (eng_in_ready),
        .in_arrival_idx  (eng_in_arrival_idx),
        .in_arrival_data (eng_in_arrival_data),
        .flush           (eng_flush),
        .flush_done      (eng_flush_done),
        .out_valid       (eng_out_valid),
        .out_ready       (eng_out_ready),
        .out_resident_idx(eng_out_resident_idx),
        .out_best_idx    (eng_out_best_idx),
        .out_best_dist   (eng_out_best_dist)
    );

    // -----------------------------------------------------------------
    // Run FSM
    // -----------------------------------------------------------------
    run_fsm #(
        .N_MAX_LOG2(N_MAX_LOG2)
    ) u_run (
        .clk                (clk),
        .rst_n              (rst_n),
        .start_pulse        (ctrl_start_pulse),
        .clear_pulse        (ctrl_clear_pulse),
        .abort_pulse        (ctrl_abort_pulse),
        .done_clear_pulse   (status_done_clear_pulse),
        .status_idle        (status_idle),
        .status_running     (status_running),
        .status_done        (status_done),
        .status_clear_busy  (status_clear_busy),
        .status_error       (status_error),
        .cycle_count        (stat_cycle_count),
        .clear_start        (clear_start_w),
        .clear_busy         (clear_busy_w),
        .clear_done         (clear_done_w),
        .harvest_enable     (harvest_enable_w),
        .axi_rd_start       (axi_rd_start),
        .axi_rd_busy        (axi_rd_busy),
        .axi_rd_done        (axi_rd_done),
        .feeder_enable      (feeder_enable),
        .feeder_reset_count (feeder_reset_count),
        .records_pushed     (records_pushed),
        .n_points           (cfg_n_points),
        .eng_flush          (eng_flush),
        .eng_flush_done     (eng_flush_done)
    );

endmodule
