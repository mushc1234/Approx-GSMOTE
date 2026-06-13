// clear_fsm.sv
// =====================================================================
// Walks the result BRAM and zeroes the valid bit on every entry.
// Uses the valid-only write strobe so the rest of each entry is left
// alone (cheaper, and irrelevant since valid=0 means the contents are
// don't-care anyway).
//
// Triggered by `start`. Asserts `busy` while running. Pulses `done`
// when the last entry has been written.
// =====================================================================

module clear_fsm #(
    parameter int N_MAX_LOG2 = 14
) (
    input  logic                    clk,
    input  logic                    rst_n,

    input  logic                    start,
    output logic                    busy,
    output logic                    done,           // one-cycle pulse

    // -- BRAM port A (valid-only write side) --
    output logic                    bram_wr_valid_en,
    output logic [N_MAX_LOG2-1:0]   bram_wr_addr,
    output logic                    bram_wr_valid_bit
);

    typedef enum logic [1:0] {
        S_IDLE,
        S_CLEAR,
        S_DONE
    } state_t;

    state_t                 state_r;
    logic [N_MAX_LOG2-1:0]  addr_r;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state_r <= S_IDLE;
            addr_r  <= '0;
        end else begin
            unique case (state_r)
                S_IDLE: begin
                    if (start) begin
                        state_r <= S_CLEAR;
                        addr_r  <= '0;
                    end
                end
                S_CLEAR: begin
                    if (addr_r == {N_MAX_LOG2{1'b1}}) begin
                        state_r <= S_DONE;
                    end else begin
                        addr_r <= addr_r + 1'b1;
                    end
                end
                S_DONE: begin
                    state_r <= S_IDLE;
                end
                default: state_r <= S_IDLE;
            endcase
        end
    end

    always_comb begin
        busy              = (state_r == S_CLEAR);
        done              = (state_r == S_DONE);
        bram_wr_valid_en  = (state_r == S_CLEAR);
        bram_wr_addr      = addr_r;
        bram_wr_valid_bit = 1'b0;
    end

endmodule
