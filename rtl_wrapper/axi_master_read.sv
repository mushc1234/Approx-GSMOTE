// axi_master_read.sv
// =====================================================================
// Simple AXI4 master read.
//
// Configuration:
//   base_addr      : DDR base of the first record
//   n_records      : how many records to fetch
//   stride_bytes   : bytes between consecutive records (in DDR)
//   beats_per_rec  : how many AXI data-beats make one record
//                    (set to ceil((4 + D*IN_WIDTH/8) / (AXI_DATA_WIDTH/8))
//                    by the caller; we don't have a clean place to derive
//                    it here because it depends on D and IN_WIDTH.)
//
// Behaviour on `start`:
//   - issue n_records read bursts of length beats_per_rec
//   - each burst starts at base_addr + i * stride_bytes
//   - beats stream out on out_valid / out_ready with out_last asserted
//     on the final beat of each record
//   - on the last beat of the last record, asserts `done` for one cycle
//
// Designed for simplicity, not throughput: keeps at most one
// outstanding read at a time. Fine for the bring-up tests; can be
// expanded to deeper outstanding later if needed.
// =====================================================================

module axi_master_read #(
    parameter int AXI_ADDR_WIDTH = 32,
    parameter int AXI_DATA_WIDTH = 32,
    parameter int AXI_ID_WIDTH   = 1,
    parameter int MAX_BEATS_PER_REC_LOG2 = 5    // up to 32 beats / record
) (
    input  logic                          clk,
    input  logic                          rst_n,

    // Control
    input  logic                          start,
    input  logic [AXI_ADDR_WIDTH-1:0]     base_addr,
    input  logic [31:0]                   n_records,
    input  logic [31:0]                   stride_bytes,
    input  logic [MAX_BEATS_PER_REC_LOG2:0] beats_per_rec, // 1..32
    output logic                          busy,
    output logic                          done,           // one-cycle pulse on completion

    // AXI4 read address channel
    output logic [AXI_ID_WIDTH-1:0]       m_axi_arid,
    output logic [AXI_ADDR_WIDTH-1:0]     m_axi_araddr,
    output logic [7:0]                    m_axi_arlen,
    output logic [2:0]                    m_axi_arsize,
    output logic [1:0]                    m_axi_arburst,
    output logic                          m_axi_arvalid,
    input  logic                          m_axi_arready,

    // AXI4 read data channel
    /* verilator lint_off UNUSED */
    input  logic [AXI_ID_WIDTH-1:0]       m_axi_rid,
    input  logic [1:0]                    m_axi_rresp,
    /* verilator lint_on UNUSED */
    input  logic [AXI_DATA_WIDTH-1:0]     m_axi_rdata,
    input  logic                          m_axi_rlast,
    input  logic                          m_axi_rvalid,
    output logic                          m_axi_rready,

    // Downstream beat stream
    output logic                          out_valid,
    input  logic                          out_ready,
    output logic [AXI_DATA_WIDTH-1:0]     out_data,
    output logic                          out_last
);

    typedef enum logic [1:0] {
        S_IDLE,
        S_ISSUE,
        S_READ,
        S_DONE
    } state_t;

    state_t                          state_r;
    logic [AXI_ADDR_WIDTH-1:0]       cur_addr_r;
    logic [31:0]                     rec_remaining_r;

    // ARSIZE: encode AXI_DATA_WIDTH/8 as log2
    // For 32-bit data this is 3'b010 (4 bytes/beat).
    localparam logic [2:0] AR_SIZE =
        (AXI_DATA_WIDTH == 32)  ? 3'b010 :
        (AXI_DATA_WIDTH == 64)  ? 3'b011 :
        (AXI_DATA_WIDTH == 128) ? 3'b100 :
        (AXI_DATA_WIDTH == 256) ? 3'b101 : 3'b010;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state_r          <= S_IDLE;
            cur_addr_r       <= '0;
            rec_remaining_r  <= '0;
        end else begin
            unique case (state_r)
                S_IDLE: begin
                    if (start) begin
                        cur_addr_r      <= base_addr;
                        rec_remaining_r <= n_records;
                        if (n_records != 0) begin
                            state_r <= S_ISSUE;
                        end else begin
                            state_r <= S_DONE;
                        end
                    end
                end
                S_ISSUE: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        state_r <= S_READ;
                    end
                end
                S_READ: begin
                    if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin
                        if (rec_remaining_r == 32'd1) begin
                            state_r <= S_DONE;
                        end else begin
                            cur_addr_r      <= cur_addr_r + stride_bytes;
                            rec_remaining_r <= rec_remaining_r - 1'b1;
                            state_r         <= S_ISSUE;
                        end
                    end
                end
                S_DONE: begin
                    state_r <= S_IDLE;
                end
                default: state_r <= S_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // AXI signals
    // -----------------------------------------------------------------
    assign m_axi_arid    = '0;
    assign m_axi_araddr  = cur_addr_r;
    assign m_axi_arlen   = 8'(({2'b0, beats_per_rec}) - 8'b1);
    assign m_axi_arsize  = AR_SIZE;
    assign m_axi_arburst = 2'b01;   // INCR
    assign m_axi_arvalid = (state_r == S_ISSUE);

    // Throttle rdata if downstream isn't ready
    assign m_axi_rready  = (state_r == S_READ) && out_ready;

    assign out_valid     = (state_r == S_READ) && m_axi_rvalid;
    assign out_data      = m_axi_rdata;
    assign out_last      = m_axi_rlast;

    assign busy = (state_r != S_IDLE);
    assign done = (state_r == S_DONE);

endmodule
