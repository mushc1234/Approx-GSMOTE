// run_fsm.sv
// =====================================================================
// Top-level sequencer for the wrapper.
//
// Phases:
//   S_IDLE         : waiting for start_pulse or clear_pulse
//   S_CLEAR        : clear FSM is wiping valid bits in result BRAM
//   S_FETCH_AND_FEED : AXI master fetching records; feeder driving them
//                      into the engine. Stays here until all N records
//                      have been pushed via the engine's in_valid/in_ready.
//   S_FLUSH        : assert flush to engine; wait for flush_done
//   S_HARVEST_DRAIN: drain remaining harvest tuples; harvest_count
//                      should reach the expected value before exit
//                      (actually: this is implicit because the engine
//                      goes idle after flush_done and the harvest FSM
//                      will process any tuple that's already presented).
//                      To be safe we wait a few cycles after flush_done.
//   S_DONE         : set status_done; return to idle on done_clear_pulse
//
// Aggregated counters across all passes are NOT reset between runs.
// The CPU does a clear-then-T-passes sequence and reads at the end.
// =====================================================================

module run_fsm #(
    parameter int N_MAX_LOG2 = 14,
    parameter int DRAIN_TAIL_CYCLES = 16
) (
    input  logic                       clk,
    input  logic                       rst_n,

    // ---- Control from AXI-Lite ----
    input  logic                       start_pulse,
    input  logic                       clear_pulse,
    input  logic                       abort_pulse,
    input  logic                       done_clear_pulse,

    // ---- Status ----
    output logic                       status_idle,
    output logic                       status_running,
    output logic                       status_done,
    output logic                       status_clear_busy,
    output logic                       status_error,

    // ---- Free-running cycle counter (running while !idle) ----
    output logic [63:0]                cycle_count,

    // ---- Clear FSM interface ----
    output logic                       clear_start,
    input  logic                       clear_busy,
    input  logic                       clear_done,

    // ---- Harvest FSM enable ----
    output logic                       harvest_enable,

    // ---- Feeder + AXI master read controls ----
    output logic                       axi_rd_start,
    input  logic                       axi_rd_busy,
    /* verilator lint_off UNUSED */
    input  logic                       axi_rd_done,
    /* verilator lint_on UNUSED */
    output logic                       feeder_enable,
    output logic                       feeder_reset_count,
    input  logic [31:0]                records_pushed,

    input  logic [31:0]                n_points,

    // ---- Engine flush ----
    output logic                       eng_flush,
    input  logic                       eng_flush_done
);

    typedef enum logic [2:0] {
        S_IDLE,
        S_CLEAR,
        S_FEED,
        S_FLUSH,
        S_DRAIN_TAIL,
        S_DONE_HOLD
    } state_t;

    state_t state_r;
    logic [31:0] drain_tail_r;
    logic        done_sticky_r;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state_r       <= S_IDLE;
            drain_tail_r  <= '0;
            done_sticky_r <= 1'b0;
        end else begin
            // done bit clears on CPU W1C ack
            if (done_clear_pulse) done_sticky_r <= 1'b0;

            unique case (state_r)
                S_IDLE: begin
                    if (clear_pulse) begin
                        state_r <= S_CLEAR;
                    end else if (start_pulse) begin
                        state_r <= S_FEED;
                    end
                end
                S_CLEAR: begin
                    if (clear_done) begin
                        state_r <= S_IDLE;
                    end
                end
                S_FEED: begin
                    if (abort_pulse) begin
                        state_r <= S_IDLE;
                    end else if (records_pushed == n_points && n_points != 0) begin
                        state_r <= S_FLUSH;
                    end
                end
                S_FLUSH: begin
                    if (eng_flush_done) begin
                        drain_tail_r <= DRAIN_TAIL_CYCLES;
                        state_r      <= S_DRAIN_TAIL;
                    end
                end
                S_DRAIN_TAIL: begin
                    if (drain_tail_r == 0) begin
                        done_sticky_r <= 1'b1;
                        state_r       <= S_DONE_HOLD;
                    end else begin
                        drain_tail_r <= drain_tail_r - 1'b1;
                    end
                end
                S_DONE_HOLD: begin
                    state_r <= S_IDLE;
                end
                default: state_r <= S_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // Outputs
    // -----------------------------------------------------------------
    always_comb begin
        status_idle       = (state_r == S_IDLE);
        status_running    = (state_r == S_FEED) || (state_r == S_FLUSH) ||
                            (state_r == S_DRAIN_TAIL);
        status_done       = done_sticky_r;
        status_clear_busy = clear_busy || (state_r == S_CLEAR);
        status_error      = 1'b0;

        clear_start        = (state_r == S_IDLE) && clear_pulse;
        axi_rd_start       = (state_r == S_IDLE) && start_pulse;
        feeder_reset_count = (state_r == S_IDLE) && start_pulse;
        feeder_enable      = (state_r == S_FEED);
        harvest_enable     = (state_r == S_FEED) || (state_r == S_FLUSH) ||
                             (state_r == S_DRAIN_TAIL);
        eng_flush          = (state_r == S_FLUSH);
    end

    // -----------------------------------------------------------------
    // Cycle counter: free-running while running
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cycle_count <= '0;
        end else if (start_pulse) begin
            cycle_count <= '0;
        end else if (status_running) begin
            cycle_count <= cycle_count + 1'b1;
        end
    end

    /* verilator lint_off UNUSED */
    logic _unused = &{1'b0, axi_rd_busy};
    logic [N_MAX_LOG2-1:0] _unused2;
    assign _unused2 = '0;
    /* verilator lint_on UNUSED */

endmodule
