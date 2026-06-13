// axi_lite_regs.sv
// =====================================================================
// AXI4-Lite slave for the GSMOTE accelerator wrapper.
//
// Implements:
//   * Control / status register file (see register map below).
//   * A read window into the result BRAM (port B). Writes to the BRAM
//     window are ignored.
//   * Backpressure on the AXI handshake -- one transaction at a time;
//     no outstanding bursts (AXI4-Lite has no bursts anyway).
//
// Address decode:
//   Low region  (offsets 0x000..0x0FF, 1 bit picks reg vs bram via
//                bit 12 in the full 16-bit address):
//     0x000  CTRL                 W1S on [0]=start_run, [1]=clear_bram, [2]=abort
//     0x004  STATUS               [0]=idle [1]=running [2]=done (W1C)
//                                 [3]=clear_busy [4]=error
//     0x008  N_POINTS             unsigned points-this-pass
//     0x00C  POINTS_PTR           DDR base address for the records
//     0x010  PASS_ID              informational, CPU-written
//     0x014  CYCLE_COUNT_LO       free-running while running=1
//     0x018  CYCLE_COUNT_HI
//     0x01C  HARVEST_COUNT
//     0x020  IMPROVEMENT_COUNT
//     0x024  SCRATCH              read/write, for CPU sanity checks
//     0x028..0x0FC  reserved (read 0)
//   High region (offsets 0x1000..):
//     RESULT_BRAM word window. Bits [N_MAX_LOG2+3:4] select entry,
//     bits [3:2] select word.
//
// The status / control flags are driven by the run FSM (which lives
// outside this module) via inputs/outputs; this module just exposes
// them on the bus.
// =====================================================================

module axi_lite_regs #(
    parameter int N_MAX_LOG2     = 14,
    parameter int AXI_ADDR_WIDTH = 16,
    parameter int AXI_DATA_WIDTH = 32
) (
    input  logic                          clk,
    input  logic                          rst_n,

    // ---- AXI4-Lite slave ----
    input  logic [AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  logic                          s_axi_awvalid,
    output logic                          s_axi_awready,
    input  logic [AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  logic [(AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  logic                          s_axi_wvalid,
    output logic                          s_axi_wready,
    output logic [1:0]                    s_axi_bresp,
    output logic                          s_axi_bvalid,
    input  logic                          s_axi_bready,

    input  logic [AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  logic                          s_axi_arvalid,
    output logic                          s_axi_arready,
    output logic [AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    output logic [1:0]                    s_axi_rresp,
    output logic                          s_axi_rvalid,
    input  logic                          s_axi_rready,

    // ---- Control pulses to the run FSM (one cycle each) ----
    output logic                          ctrl_start_pulse,
    output logic                          ctrl_clear_pulse,
    output logic                          ctrl_abort_pulse,
    output logic                          status_done_clear_pulse,

    // ---- Status / config registers (driven from run FSM) ----
    input  logic                          status_idle,
    input  logic                          status_running,
    input  logic                          status_done,
    input  logic                          status_clear_busy,
    input  logic                          status_error,

    output logic [31:0]                   cfg_n_points,
    output logic [31:0]                   cfg_points_ptr,
    output logic [31:0]                   cfg_pass_id,

    input  logic [63:0]                   stat_cycle_count,
    input  logic [31:0]                   stat_harvest_count,
    input  logic [31:0]                   stat_improvement_count,

    // ---- Result BRAM read port (port B) ----
    output logic                          bram_b_rd_en,
    output logic [N_MAX_LOG2-1:0]         bram_b_rd_entry,
    output logic [1:0]                    bram_b_rd_word,
    input  logic [31:0]                   bram_b_rd_data
);

    localparam logic [AXI_ADDR_WIDTH-1:0] BRAM_BASE = AXI_ADDR_WIDTH'('h1000);

    // -----------------------------------------------------------------
    // Register file
    // -----------------------------------------------------------------
    logic [31:0] reg_n_points;
    logic [31:0] reg_points_ptr;
    logic [31:0] reg_pass_id;
    logic [31:0] reg_scratch;

    assign cfg_n_points   = reg_n_points;
    assign cfg_points_ptr = reg_points_ptr;
    assign cfg_pass_id    = reg_pass_id;

    // -----------------------------------------------------------------
    // Write channel state
    //
    // Two-phase: capture awaddr+wdata, then assert bvalid until accepted.
    // -----------------------------------------------------------------
    typedef enum logic [1:0] {
        AW_IDLE,
        AW_WAIT,
        AW_RESP
    } aw_state_t;

    aw_state_t                       aw_state_r;
    logic [AXI_ADDR_WIDTH-1:0]       wr_addr_r;
    logic [AXI_DATA_WIDTH-1:0]       wr_data_r;

    // High-region bit: bit 12 selects BRAM window.
    logic                            wr_is_bram;
    assign wr_is_bram = (wr_addr_r >= BRAM_BASE);

    // Decoded reg address (within low region, byte addr aligned to 4)
    logic [7:0]                      wr_reg_off;
    assign wr_reg_off = wr_addr_r[7:0] & 8'hFC;

    /* verilator lint_off UNUSED */
    logic _unused_wr_addr = &{1'b0, wr_addr_r};
    /* verilator lint_on UNUSED */

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            aw_state_r              <= AW_IDLE;
            wr_addr_r               <= '0;
            wr_data_r               <= '0;
            reg_n_points            <= '0;
            reg_points_ptr          <= '0;
            reg_pass_id             <= '0;
            reg_scratch             <= '0;
            ctrl_start_pulse        <= 1'b0;
            ctrl_clear_pulse        <= 1'b0;
            ctrl_abort_pulse        <= 1'b0;
            status_done_clear_pulse <= 1'b0;
        end else begin
            // Default the one-cycle pulses
            ctrl_start_pulse        <= 1'b0;
            ctrl_clear_pulse        <= 1'b0;
            ctrl_abort_pulse        <= 1'b0;
            status_done_clear_pulse <= 1'b0;

            unique case (aw_state_r)
                AW_IDLE: begin
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        wr_addr_r  <= s_axi_awaddr;
                        wr_data_r  <= s_axi_wdata;
                        aw_state_r <= AW_RESP;
                    end else if (s_axi_awvalid) begin
                        wr_addr_r  <= s_axi_awaddr;
                        aw_state_r <= AW_WAIT;
                    end
                end
                AW_WAIT: begin
                    if (s_axi_wvalid) begin
                        wr_data_r  <= s_axi_wdata;
                        aw_state_r <= AW_RESP;
                    end
                end
                AW_RESP: begin
                    // Effect the write
                    if (!wr_is_bram) begin
                        unique case (wr_reg_off)
                            8'h00: begin   // CTRL
                                if (wr_data_r[0]) ctrl_start_pulse <= 1'b1;
                                if (wr_data_r[1]) ctrl_clear_pulse <= 1'b1;
                                if (wr_data_r[2]) ctrl_abort_pulse <= 1'b1;
                            end
                            8'h04: begin   // STATUS: W1C on done bit
                                if (wr_data_r[2]) status_done_clear_pulse <= 1'b1;
                            end
                            8'h08: reg_n_points   <= wr_data_r;
                            8'h0C: reg_points_ptr <= wr_data_r;
                            8'h10: reg_pass_id    <= wr_data_r;
                            8'h24: reg_scratch    <= wr_data_r;
                            default: ; // ignored
                        endcase
                    end
                    // BRAM writes are silently ignored (read-only window)
                    if (s_axi_bready) begin
                        aw_state_r <= AW_IDLE;
                    end
                end
                default: aw_state_r <= AW_IDLE;
            endcase
        end
    end

    assign s_axi_awready = (aw_state_r == AW_IDLE);
    assign s_axi_wready  = (aw_state_r == AW_IDLE && s_axi_awvalid) ||
                           (aw_state_r == AW_WAIT);
    assign s_axi_bvalid  = (aw_state_r == AW_RESP);
    assign s_axi_bresp   = 2'b00; // OKAY

    /* verilator lint_off UNUSED */
    logic _unused_wstrb = &{1'b0, s_axi_wstrb};
    /* verilator lint_on UNUSED */

    // -----------------------------------------------------------------
    // Read channel state
    //
    // Two-phase: accept ar, look up data, drive rvalid until ready.
    // BRAM reads have one cycle of latency through port B, plus
    // one more cycle for the combinational mux inside the BRAM.
    // We add a small RD pipeline to account for that.
    // -----------------------------------------------------------------
    typedef enum logic [1:0] {
        AR_IDLE,
        AR_BRAM_WAIT_1,
        AR_BRAM_WAIT_2,
        AR_RESP
    } ar_state_t;

    ar_state_t                       ar_state_r;
    logic [AXI_ADDR_WIDTH-1:0]       rd_addr_r;
    logic [AXI_DATA_WIDTH-1:0]       rd_data_r;

    logic                            rd_is_bram;
    assign rd_is_bram = (rd_addr_r >= BRAM_BASE);

    logic [7:0]                      rd_reg_off;
    assign rd_reg_off = rd_addr_r[7:0] & 8'hFC;

    /* verilator lint_off UNUSED */
    logic _unused_rd_addr = &{1'b0, rd_addr_r};
    /* verilator lint_on UNUSED */

    // -----------------------------------------------------------------
    // Synchronous register read
    // -----------------------------------------------------------------
    logic [31:0]                     reg_read_data;
    always_comb begin
        unique case (rd_reg_off)
            8'h00: reg_read_data = 32'h0;            // CTRL reads 0 (self-clear)
            8'h04: reg_read_data = {27'b0,
                                    status_error,
                                    status_clear_busy,
                                    status_done,
                                    status_running,
                                    status_idle};
            8'h08: reg_read_data = reg_n_points;
            8'h0C: reg_read_data = reg_points_ptr;
            8'h10: reg_read_data = reg_pass_id;
            8'h14: reg_read_data = stat_cycle_count[31:0];
            8'h18: reg_read_data = stat_cycle_count[63:32];
            8'h1C: reg_read_data = stat_harvest_count;
            8'h20: reg_read_data = stat_improvement_count;
            8'h24: reg_read_data = reg_scratch;
            default: reg_read_data = 32'h0;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ar_state_r <= AR_IDLE;
            rd_addr_r  <= '0;
            rd_data_r  <= '0;
        end else begin
            unique case (ar_state_r)
                AR_IDLE: begin
                    if (s_axi_arvalid) begin
                        rd_addr_r <= s_axi_araddr;
                        if (s_axi_araddr >= BRAM_BASE) begin // Fixed: Replaced [12]
                            ar_state_r <= AR_BRAM_WAIT_1;
                        end else begin
                            ar_state_r <= AR_BRAM_WAIT_2;
                        end
                    end
                end
                AR_BRAM_WAIT_1: begin
                    // BRAM port B has one read-data register inside, so
                    // wait one cycle before sampling.
                    ar_state_r <= AR_BRAM_WAIT_2;
                end
                AR_BRAM_WAIT_2: begin
                    rd_data_r  <= rd_is_bram ? bram_b_rd_data : reg_read_data;
                    ar_state_r <= AR_RESP;
                end
                AR_RESP: begin
                    if (s_axi_rready) begin
                        ar_state_r <= AR_IDLE;
                    end
                end
                default: ar_state_r <= AR_IDLE;
            endcase
        end
    end

    assign s_axi_arready = (ar_state_r == AR_IDLE);
    assign s_axi_rvalid  = (ar_state_r == AR_RESP);
    assign s_axi_rresp   = 2'b00;
    assign s_axi_rdata   = rd_data_r;

    // BRAM port B drive: assert b_rd_en in AR_IDLE-cycle when ar accepted.
    // The handshake captures the address in the same cycle as accept.
    assign bram_b_rd_en = (ar_state_r == AR_IDLE) && s_axi_arvalid && (s_axi_araddr >= BRAM_BASE);
    
    logic [AXI_ADDR_WIDTH-1:0] bram_offset;
    assign bram_offset = s_axi_araddr - BRAM_BASE;
    assign bram_b_rd_entry = bram_offset[N_MAX_LOG2+3:4];
    
    assign bram_b_rd_word  = s_axi_araddr[3:2];

endmodule
