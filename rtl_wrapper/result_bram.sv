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
    parameter int N_MAX_LOG2     = 14,                // 16384 entries
    parameter int IDX_WIDTH      = 16,
    parameter int ACC_STORE_WIDTH = 48                // bits stored for best_dist
) (
    input  logic                       clk,
    input  logic                       rst_n,

    // -------- Port A: wrapper side (harvest + clear) --------
    input  logic [N_MAX_LOG2-1:0]      a_rd_addr,
    output logic                       a_rd_valid_bit,
    output logic [IDX_WIDTH-1:0]       a_rd_resident_idx,
    output logic [IDX_WIDTH-1:0]       a_rd_best_idx,
    output logic [ACC_STORE_WIDTH-1:0] a_rd_best_dist,

    input  logic                       a_wr_full_en,   // write all fields
    input  logic                       a_wr_valid_en,  // write only valid bit
    input  logic [N_MAX_LOG2-1:0]      a_wr_addr,
    input  logic                       a_wr_valid_bit,
    input  logic [IDX_WIDTH-1:0]       a_wr_resident_idx,
    input  logic [IDX_WIDTH-1:0]       a_wr_best_idx,
    input  logic [ACC_STORE_WIDTH-1:0] a_wr_best_dist,

    // -------- Port B: CPU/MMIO side (read-only word access) --------
    input  logic                       b_rd_en,
    input  logic [N_MAX_LOG2-1:0]      b_rd_entry,
    input  logic [1:0]                 b_rd_word,
    output logic [31:0]                b_rd_data
);

    // -----------------------------------------------------------------
    // Storage layout: pack into a single wide word per entry. Bit
    // positions match the MMIO word layout above for symmetry.
    // -----------------------------------------------------------------
    localparam int ENTRY_W = 128;

    // Bit slices within the 128-bit word
    localparam int DIST_LO     = 0;
    localparam int DIST_HI     = ACC_STORE_WIDTH - 1;       // <= 63 for ACC_STORE_WIDTH<=48
    localparam int RESIDENT_LO = 64;
    localparam int RESIDENT_HI = 64 + IDX_WIDTH - 1;
    localparam int BEST_LO     = 80;
    localparam int BEST_HI     = 80 + IDX_WIDTH - 1;
    localparam int VALID_BIT   = 96;

    logic [ENTRY_W-1:0] mem       [0:(1<<N_MAX_LOG2)-1]; 
    logic               mem_valid [0:(1<<N_MAX_LOG2)-1];

    // -----------------------------------------------------------------
    // Port A read: registered (one-cycle latency, BRAM-style)
    // -----------------------------------------------------------------
    logic [ENTRY_W-1:0] a_rd_word;

    always_ff @(posedge clk) begin
        a_rd_word      <= mem[a_rd_addr];
        a_rd_valid_bit <= mem_valid[a_rd_addr]; 
    end
    // Remove a_rd_valid_bit from the always_comb block below it

    always_comb begin
        a_rd_resident_idx = a_rd_word[RESIDENT_HI:RESIDENT_LO];
        a_rd_best_idx     = a_rd_word[BEST_HI:BEST_LO];
        a_rd_best_dist    = a_rd_word[DIST_HI:DIST_LO];
    end

    // -----------------------------------------------------------------
    // Port A write
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (a_wr_full_en) begin
            // Write data to main memory
            mem[a_wr_addr][DIST_HI:DIST_LO]         <= a_wr_best_dist;
            mem[a_wr_addr][RESIDENT_HI:RESIDENT_LO] <= a_wr_resident_idx;
            mem[a_wr_addr][BEST_HI:BEST_LO]         <= a_wr_best_idx;
            // Write flag to valid memory
            mem_valid[a_wr_addr]                    <= a_wr_valid_bit;
        end
        else if (a_wr_valid_en) begin
            mem_valid[a_wr_addr] <= a_wr_valid_bit;
        end
    end

    // -----------------------------------------------------------------
    // Port B read: registered word access, MMIO-style
    // -----------------------------------------------------------------
    logic [ENTRY_W-1:0] b_rd_word_r;
    logic [1:0]         b_rd_word_sel_r;
    logic               b_rd_en_r;
    logic               b_rd_valid_r;

    always_ff @(posedge clk) begin
        b_rd_en_r       <= b_rd_en;
        b_rd_word_sel_r <= b_rd_word;
        if (b_rd_en) begin
            b_rd_word_r <= mem[b_rd_entry];
            b_rd_valid_r <= mem_valid[b_rd_entry]; // Add this line
        end
    end

    always_comb begin
        unique case (b_rd_word_sel_r)
            2'd0: b_rd_data = b_rd_word_r[31:0];
            // word 1: upper bits of dist (bits 32..min(63, DIST_HI))
            //   we currently keep ACC_STORE_WIDTH<=48, so dist fits in bits 32..47
            2'd1: b_rd_data = {16'b0, b_rd_word_r[47:32]};
            2'd2: b_rd_data = {b_rd_word_r[BEST_HI:BEST_LO],
                               b_rd_word_r[RESIDENT_HI:RESIDENT_LO]};
            2'd3: b_rd_data = {31'b0, b_rd_valid_r};
            default: b_rd_data = '0;
        endcase
    end

endmodule
