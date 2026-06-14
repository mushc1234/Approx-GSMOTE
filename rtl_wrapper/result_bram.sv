// result_bram.sv
// =====================================================================
// Result memory for the GSMOTE/sigwin accelerator wrapper.
//
// One entry per resident_idx. Holds the best (best_idx, best_dist) seen
// across all passes plus a valid bit indicating whether anything has
// been written for that resident yet. Used by the harvest FSM for
// compare-and-write (multi-pass aggregation) and by the MicroBlaze for
// MMIO readback via the AXI-Lite slave.
//
// Layout per entry (128 bits = 4 x 32-bit words, MMIO-friendly):
//   word 0 [31:0]   best_dist[31:0]
//   word 1 [15:0]   best_dist[47:32]    (only bits [39:32] used today)
//          [31:16]  reserved
//   word 2 [15:0]   resident_idx
//          [31:16]  best_idx
//   word 3 [0]      valid
//          [31:1]   reserved
//
// Port A (wrapper side):
//   - dedicated read addr / read data (one-cycle latency)
//   - write enable + addr + data
//   - separate write strobe for `valid` (used by clear FSM to wipe valid
//     without disturbing the rest of the entry)
//
// Port B (CPU/MMIO side):
//   - 32-bit word read interface. Address is byte addr within the BRAM
//     window; lower 4 bits select word, upper bits select entry.
//   - read-only from this port. Wrapper writes are not arbitrated against
//     CPU reads because the CPU is expected to leave the BRAM alone
//     while running=1 (enforced in the slave layer, not here).
// =====================================================================

module result_bram #(
    parameter int N_MAX_LOG2     = 14,
    parameter int IDX_WIDTH      = 16,
    parameter int ACC_STORE_WIDTH = 48
) (
    input  logic                   clk,
    input  logic                   rst_n,

    // -------- Port A: wrapper side (harvest + clear) --------
    input  logic [N_MAX_LOG2-1:0]      a_rd_addr,
    output logic                       a_rd_valid_bit,
    output logic [IDX_WIDTH-1:0]       a_rd_resident_idx,
    output logic [IDX_WIDTH-1:0]       a_rd_best_idx,
    output logic [ACC_STORE_WIDTH-1:0] a_rd_best_dist,

    input  logic                       a_wr_full_en,
    input  logic                       a_wr_valid_en,
    input  logic [N_MAX_LOG2-1:0]      a_wr_addr,
    input  logic                       a_wr_valid_bit,
    input  logic [IDX_WIDTH-1:0]       a_wr_resident_idx,
    input  logic [IDX_WIDTH-1:0]       a_wr_best_idx,
    input  logic [ACC_STORE_WIDTH-1:0] a_wr_best_dist,

    // -------- Port B: CPU/MMIO side --------
    input  logic                       b_rd_en,
    input  logic [N_MAX_LOG2-1:0]      b_rd_entry,
    input  logic [1:0]                 b_rd_word,
    output logic [31:0]                b_rd_data
);

    localparam int ENTRY_W = 128;
    localparam int DIST_LO     = 0;
    localparam int DIST_HI     = ACC_STORE_WIDTH - 1;
    localparam int RESIDENT_LO = 64;
    localparam int RESIDENT_HI = 64 + IDX_WIDTH - 1;
    localparam int BEST_LO     = 80;
    localparam int BEST_HI     = 80 + IDX_WIDTH - 1;
    localparam int VALID_BIT   = 96; // Packed into main word

    logic [ENTRY_W-1:0] mem [0:(1<<N_MAX_LOG2)-1];

    // Port A Read
    always_ff @(posedge clk) begin
        logic [ENTRY_W-1:0] rd_data;
        rd_data <= mem[a_rd_addr];
        a_rd_valid_bit    <= rd_data[VALID_BIT];
        a_rd_resident_idx <= rd_data[RESIDENT_HI:RESIDENT_LO];
        a_rd_best_idx     <= rd_data[BEST_HI:BEST_LO];
        a_rd_best_dist    <= rd_data[DIST_HI:DIST_LO];
    end

    // Port A Write: Packed into single 128-bit assignment
    always_ff @(posedge clk) begin
        if (a_wr_full_en) begin
            mem[a_wr_addr]                 <= '0; // Clear reserved bits
            mem[a_wr_addr][DIST_HI:DIST_LO]         <= a_wr_best_dist;
            mem[a_wr_addr][RESIDENT_HI:RESIDENT_LO] <= a_wr_resident_idx;
            mem[a_wr_addr][BEST_HI:BEST_LO]         <= a_wr_best_idx;
            mem[a_wr_addr][VALID_BIT]               <= a_wr_valid_bit;
        end
        else if (a_wr_valid_en) begin
            mem[a_wr_addr][VALID_BIT] <= a_wr_valid_bit;
        end
    end

    // Port B Read: Accesses packed memory directly
    logic [ENTRY_W-1:0] b_rd_word_r;
    always_ff @(posedge clk) begin
        if (b_rd_en) b_rd_word_r <= mem[b_rd_entry];
    end

    always_comb begin
        unique case (b_rd_word)
            2'd0:    b_rd_data = b_rd_word_r[31:0];
            2'd1:    b_rd_data = {16'b0, b_rd_word_r[47:32]};
            2'd2:    b_rd_data = {b_rd_word_r[BEST_HI:BEST_LO], b_rd_word_r[RESIDENT_HI:RESIDENT_LO]};
            2'd3:    b_rd_data = {31'b0, b_rd_word_r[VALID_BIT]};
            default: b_rd_data = '0;
        endcase
    end
endmodule