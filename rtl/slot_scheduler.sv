// slot_scheduler.sv
// =====================================================================
// Round-robin eviction scheduler for the W-slot window engine.
//
// Behaviour:
//   On each accepted arrival (in_valid && in_ready), the scheduler
//   broadcasts the arrival to all W slots and increments an "arrivals
//   since last eviction" counter. When the counter reaches a
//   threshold (currently: one eviction per arrival, but parametric
//   via EVICT_INTERVAL), the scheduler enters its eviction sequence:
//
//     1. DRAIN  -- stall in_ready for LATENCY cycles so all in-flight
//                  pipelined comparisons complete and update their
//                  slots' running-best registers.
//     2. HARVEST -- sample the target slot's (resident_idx, best_idx,
//                   best_dist) and emit on the output stream.
//                   Stalls if downstream not ready.
//     3. LOAD   -- assert load_valid on the target slot with the next
//                  arrival's data; advance the round-robin pointer.
//     4. STREAM -- back to normal arrival broadcast.
//
//   At end-of-stream, the host asserts `flush`; the scheduler walks
//   all W slots through DRAIN+HARVEST without LOAD, then asserts
//   flush_done.
//
// Parameters of note:
//   W              -- number of slots (power of 2 strongly preferred
//                     for round-robin pointer arithmetic to be cheap)
//   LATENCY        -- distance unit pipeline depth + 1 (the extra
//                     cycle is for the running-best register to latch
//                     the emerging cmp_valid). Must match the slot's
//                     actual depth or stale data will leak.
//   EVICT_INTERVAL -- arrivals processed between evictions. =1 means
//                     evict on every arrival (correct sliding-window
//                     semantics). Larger values trade window freshness
//                     for higher arrival throughput; the engine still
//                     produces correct results, just with a slightly
//                     different sliding behaviour. Default 1.
// =====================================================================

module slot_scheduler #(
    parameter int W              = 8,
    parameter int D              = 32,
    parameter int IN_WIDTH       = 16,
    parameter int ACC_WIDTH      = 40,
    parameter int IDX_WIDTH      = 16,
    parameter int LATENCY        = 7,      // distance_unit_pipelined depth + 1
    parameter int EVICT_INTERVAL = 1
) (
    input  logic                              clk,
    input  logic                              rst_n,

    // ---- Host arrival input stream ----
    input  logic                              in_valid,
    output logic                              in_ready,
    input  logic [IDX_WIDTH-1:0]              in_arrival_idx,
    input  logic [D*IN_WIDTH-1:0]             in_arrival_data,

    // ---- End-of-stream flush ----
    input  logic                              flush,
    output logic                              flush_done,

    // ---- Per-slot load (one-hot: only one slot loads per LOAD cycle) ----
    output logic [W-1:0]                      slot_load_valid,
    output logic [IDX_WIDTH-1:0]              slot_load_idx,
    output logic [D*IN_WIDTH-1:0]             slot_load_data,

    // ---- Arrival broadcast to all slots ----
    output logic                              slot_arrival_valid,
    output logic [IDX_WIDTH-1:0]              slot_arrival_idx,
    output logic [D*IN_WIDTH-1:0]             slot_arrival_data,

    // ---- Per-slot state read-back (from slots; for harvest) ----
    input  logic [IDX_WIDTH-1:0]              slot_resident_idx [W],
    input  logic [IDX_WIDTH-1:0]              slot_best_idx     [W],
    input  logic [ACC_WIDTH-1:0]              slot_best_dist    [W],

    // ---- Harvest output stream to host ----
    output logic                              out_valid,
    input  logic                              out_ready,
    output logic [IDX_WIDTH-1:0]              out_resident_idx,
    output logic [IDX_WIDTH-1:0]              out_best_idx,
    output logic [ACC_WIDTH-1:0]              out_best_dist
);

    // -----------------------------------------------------------------
    // Pointer widths and useful local params
    // -----------------------------------------------------------------
    localparam int PTR_W   = (W <= 1) ? 1 : $clog2(W);
    localparam int CNT_W   = (LATENCY <= 1) ? 1 : $clog2(LATENCY + 1);
    localparam int IVL_W   = (EVICT_INTERVAL <= 1) ? 1 : $clog2(EVICT_INTERVAL + 1);

    // -----------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------
    typedef enum logic [2:0] {
        S_STREAM,        // accepting arrivals, broadcasting to all slots
        S_DRAIN,         // waiting LATENCY cycles for in-flight cmps to settle
        S_HARVEST,       // sampling target slot's running-best onto out stream
        S_LOAD,          // asserting load_valid on target slot with new arrival
        S_FLUSH_DRAIN,   // flush variant of DRAIN
        S_FLUSH_HARVEST, // flush variant of HARVEST (no LOAD follows)
        S_FLUSH_DONE     // all slots flushed; assert flush_done until released
    } state_t;

    state_t state_r, state_n;

    // Round-robin eviction pointer
    logic [PTR_W-1:0] evict_ptr_r, evict_ptr_n;

    // Arrivals-since-last-eviction counter
    logic [IVL_W-1:0] arr_count_r, arr_count_n;

    // DRAIN cycle counter (counts down from LATENCY)
    logic [CNT_W-1:0] drain_cnt_r, drain_cnt_n;

    // Flush walk counter (counts up from 0 to W during flush)
    logic [PTR_W:0]   flush_cnt_r, flush_cnt_n;  // one extra bit so W is representable

    // Latched arrival data for use during LOAD (since in_ready deasserts
    // during DRAIN/HARVEST/LOAD, the source has already left the bus)
    logic [IDX_WIDTH-1:0]    pending_idx_r,  pending_idx_n;
    logic [D*IN_WIDTH-1:0]   pending_data_r, pending_data_n;
    logic                    pending_valid_r, pending_valid_n;

    // -----------------------------------------------------------------
    // Combinational next-state and output logic
    // -----------------------------------------------------------------
    always_comb begin
        // Local computed value: which slot to read during FLUSH_HARVEST.
        // Declared at the top to satisfy SystemVerilog's automatic-decl rules.
        automatic logic [PTR_W-1:0] flush_target = evict_ptr_r + flush_cnt_r[PTR_W-1:0];

        // Defaults: hold state, no outputs
        state_n         = state_r;
        evict_ptr_n     = evict_ptr_r;
        arr_count_n     = arr_count_r;
        drain_cnt_n     = drain_cnt_r;
        flush_cnt_n     = flush_cnt_r;
        pending_idx_n   = pending_idx_r;
        pending_data_n  = pending_data_r;
        pending_valid_n = pending_valid_r;

        in_ready           = 1'b0;
        slot_arrival_valid = 1'b0;
        slot_arrival_idx   = '0;
        slot_arrival_data  = '0;
        slot_load_valid    = '0;
        slot_load_idx      = '0;
        slot_load_data     = '0;
        out_valid          = 1'b0;
        out_resident_idx   = '0;
        out_best_idx       = '0;
        out_best_dist      = '0;
        flush_done         = 1'b0;

        unique case (state_r)
            // -----------------------------------------------------------
            S_STREAM: begin
                // Accept arrivals; broadcast to all slots
                if (flush) begin
                    // Begin flush sequence: drain whatever is in flight,
                    // then walk all W slots through harvest.
                    state_n     = S_FLUSH_DRAIN;
                    drain_cnt_n = CNT_W'(LATENCY);
                    flush_cnt_n = '0;
                end else begin
                    in_ready           = 1'b1;
                    slot_arrival_valid = in_valid;
                    slot_arrival_idx   = in_arrival_idx;
                    slot_arrival_data  = in_arrival_data;

                    if (in_valid) begin
                        // Latch the arrival in case we evict (we'll load
                        // it into the freshly-vacated slot after harvest)
                        pending_idx_n   = in_arrival_idx;
                        pending_data_n  = in_arrival_data;
                        pending_valid_n = 1'b1;

                        if (arr_count_r == IVL_W'(EVICT_INTERVAL - 1)) begin
                            // Reached eviction interval; enter drain.
                            arr_count_n = '0;
                            state_n     = S_DRAIN;
                            drain_cnt_n = CNT_W'(LATENCY);
                        end else begin
                            arr_count_n = arr_count_r + 1'b1;
                        end
                    end
                end
            end

            // -----------------------------------------------------------
            S_DRAIN: begin
                // Stall arrivals. Wait for in-flight comparisons to
                // settle into their slots' running-best registers.
                if (drain_cnt_r == '0) begin
                    state_n = S_HARVEST;
                end else begin
                    drain_cnt_n = drain_cnt_r - 1'b1;
                end
            end

            // -----------------------------------------------------------
            S_HARVEST: begin
                // Drive the target slot's state on the output stream.
                out_valid        = 1'b1;
                out_resident_idx = slot_resident_idx[evict_ptr_r];
                out_best_idx     = slot_best_idx    [evict_ptr_r];
                out_best_dist    = slot_best_dist   [evict_ptr_r];
                if (out_ready) begin
                    state_n = S_LOAD;
                end
            end

            // -----------------------------------------------------------
            S_LOAD: begin
                // Assert load_valid on the evicted slot with the pending
                // arrival's data. One-hot encoding: only target slot loads.
                slot_load_valid              = '0;
                slot_load_valid[evict_ptr_r] = pending_valid_r;
                slot_load_idx                = pending_idx_r;
                slot_load_data               = pending_data_r;

                // Advance round-robin pointer (wraps via modular arithmetic
                // because PTR_W = clog2(W), assuming W is a power of 2;
                // for non-pow2 W we'd need an explicit compare-and-reset).
                if (W == (1 << PTR_W)) begin
                    evict_ptr_n = evict_ptr_r + 1'b1;
                end else begin
                    evict_ptr_n = (evict_ptr_r == PTR_W'(W - 1))
                                ? '0
                                : evict_ptr_r + 1'b1;
                end

                pending_valid_n = 1'b0;
                state_n         = S_STREAM;
            end

            // -----------------------------------------------------------
            S_FLUSH_DRAIN: begin
                // Same drain as regular eviction; sets up for FLUSH_HARVEST.
                if (drain_cnt_r == '0) begin
                    state_n = S_FLUSH_HARVEST;
                end else begin
                    drain_cnt_n = drain_cnt_r - 1'b1;
                end
            end

            // -----------------------------------------------------------
            S_FLUSH_HARVEST: begin
                // Harvest slot pointed to by (evict_ptr_r + flush_cnt_r) mod W.
                // Note: we walk by flush_cnt_r rather than advancing
                // evict_ptr_r so that, after flush_done, the round-robin
                // pointer is preserved for any subsequent stream.
                out_valid        = 1'b1;
                out_resident_idx = slot_resident_idx[flush_target];
                out_best_idx     = slot_best_idx    [flush_target];
                out_best_dist    = slot_best_dist   [flush_target];

                if (out_ready) begin
                    if (flush_cnt_r == (PTR_W+1)'(W - 1)) begin
                        state_n = S_FLUSH_DONE;
                    end else begin
                        flush_cnt_n = flush_cnt_r + 1'b1;
                    end
                end
            end

            // -----------------------------------------------------------
            S_FLUSH_DONE: begin
                // Assert flush_done until host deasserts flush.
                flush_done = 1'b1;
                if (!flush) begin
                    state_n = S_STREAM;
                    flush_cnt_n = '0;
                end
            end

            default: state_n = S_STREAM;
        endcase
    end

    // -----------------------------------------------------------------
    // State register
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state_r         <= S_STREAM;
            evict_ptr_r     <= '0;
            arr_count_r     <= '0;
            drain_cnt_r     <= '0;
            flush_cnt_r     <= '0;
            pending_idx_r   <= '0;
            pending_data_r  <= '0;
            pending_valid_r <= 1'b0;
        end else begin
            state_r         <= state_n;
            evict_ptr_r     <= evict_ptr_n;
            arr_count_r     <= arr_count_n;
            drain_cnt_r     <= drain_cnt_n;
            flush_cnt_r     <= flush_cnt_n;
            pending_idx_r   <= pending_idx_n;
            pending_data_r  <= pending_data_n;
            pending_valid_r <= pending_valid_n;
        end
    end

endmodule
