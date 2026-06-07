// window_engine.sv
// =====================================================================
// Integrated top-level: W slots + arrival aggregator + slot scheduler.
//
// Adds the arrival aggregator + pending-best register to the previous
// design, so each new resident's running-best is seeded with the best
// neighbour found among the W current residents (rather than starting
// at sentinel). This closes the boundary-effect gap with the Python
// reference -- exact-match rate against final_nn.csv should jump from
// ~56% to near-100% on the bring-up test case.
//
// Pipeline timing reference (for the curious):
//   T:                 arrival X accepted; broadcast to all slots
//   T+1..T+LATENCY-1:  scheduler drains; pipeline propagates
//   T+LATENCY:         slots emit cmp_valid for X (one-cycle pulse);
//                       aggregator combinationally finds min;
//                       scheduler enters HARVEST
//   T+LATENCY+1:       aggregator output registered (agg_out_valid=1);
//                       pending-best register latches the min;
//                       scheduler enters LOAD
//   T+LATENCY+2:       slot S0 loads X; slot reads load_best from
//                       pending-best register; the resident X now starts
//                       its life with best = (min dist, winning resident)
//                       from the aggregation
//   T+LATENCY+3:       scheduler returns to STREAM, in_ready=1
// =====================================================================

module window_engine #(
    parameter int W              = 8,
    parameter int D              = 32,
    parameter int IN_WIDTH       = 16,
    parameter int ACC_WIDTH      = 40,
    parameter int IDX_WIDTH      = 16,
    parameter int EVICT_INTERVAL = 1
) (
    input  logic                              clk,
    input  logic                              rst_n,

    input  logic                              in_valid,
    output logic                              in_ready,
    input  logic [IDX_WIDTH-1:0]              in_arrival_idx,
    input  logic [D*IN_WIDTH-1:0]             in_arrival_data,

    input  logic                              flush,
    output logic                              flush_done,

    output logic                              out_valid,
    input  logic                              out_ready,
    output logic [IDX_WIDTH-1:0]              out_resident_idx,
    output logic [IDX_WIDTH-1:0]              out_best_idx,
    output logic [ACC_WIDTH-1:0]              out_best_dist
);

    // Pipeline depth seen by the scheduler: distance unit ($clog2(D)+2)
    // plus one cycle for the slot's running-best register to latch.
    localparam int SLOT_LATENCY = $clog2(D) + 3;
    localparam int TAG_WIDTH    = IDX_WIDTH * 2;

    // -----------------------------------------------------------------
    // Scheduler-driven broadcast and per-slot load
    // -----------------------------------------------------------------
    logic                       sched_arrival_valid;
    logic [IDX_WIDTH-1:0]       sched_arrival_idx;
    logic [D*IN_WIDTH-1:0]      sched_arrival_data;
    logic [W-1:0]               sched_load_valid;
    logic [IDX_WIDTH-1:0]       sched_load_idx;
    logic [D*IN_WIDTH-1:0]      sched_load_data;

    // -----------------------------------------------------------------
    // Slot status (to scheduler) and cmp outputs (to aggregator)
    // -----------------------------------------------------------------
    logic                       slot_resident_valid [W];
    logic [IDX_WIDTH-1:0]       slot_resident_idx   [W];
    logic [IDX_WIDTH-1:0]       slot_best_idx       [W];
    logic [ACC_WIDTH-1:0]       slot_best_dist      [W];
    logic                       slot_cmp_valid      [W];
    logic [ACC_WIDTH-1:0]       slot_cmp_dist       [W];
    logic [TAG_WIDTH-1:0]       slot_cmp_tag        [W];

    /* verilator lint_off UNUSED */
    logic [W-1:0]               unused_resident_valid_bits;
    always_comb begin
        for (int i = 0; i < W; i++)
            unused_resident_valid_bits[i] = slot_resident_valid[i];
    end
    /* verilator lint_on UNUSED */

    // -----------------------------------------------------------------
    // Aggregator
    // -----------------------------------------------------------------
    logic                       agg_out_valid;
    logic [ACC_WIDTH-1:0]       agg_out_min_dist;
    logic [TAG_WIDTH-1:0]       agg_out_min_tag;

    arrival_aggregator #(
        .W       (W),
        .ACC_WIDTH(ACC_WIDTH),
        .TAG_WIDTH(TAG_WIDTH)
    ) u_agg (
        .clk           (clk),
        .rst_n         (rst_n),
        .slot_cmp_valid(slot_cmp_valid),
        .slot_cmp_dist (slot_cmp_dist),
        .slot_cmp_tag  (slot_cmp_tag),
        .out_valid     (agg_out_valid),
        .out_min_dist  (agg_out_min_dist),
        .out_min_tag   (agg_out_min_tag)
    );

    logic [ACC_WIDTH-1:0]       agg_out_min_dist_v;

    assign agg_out_min_dist_v = agg_out_valid ? agg_out_min_dist
                                              : {ACC_WIDTH{1'b1}};

    // -----------------------------------------------------------------
    // W slot instances
    // -----------------------------------------------------------------
    genvar s;
    generate
        for (s = 0; s < W; s++) begin : slot_inst
            window_slot #(
                .D       (D),
                .IN_WIDTH(IN_WIDTH),
                .ACC_WIDTH(ACC_WIDTH),
                .IDX_WIDTH(IDX_WIDTH)
            ) u_slot (
                .clk                (clk),
                .rst_n              (rst_n),

                .load_valid         (sched_load_valid[s]),
                .load_idx           (sched_load_idx),
                .load_data          (sched_load_data),

                
                .agg_best_dist      (agg_out_min_dist_v),
                .agg_best_tag       (agg_best_tag),

                .arrival_valid      (sched_arrival_valid),
                .arrival_idx        (sched_arrival_idx),
                .arrival_data       (sched_arrival_data),

                .resident_valid     (slot_resident_valid[s]),
                .resident_idx       (slot_resident_idx  [s]),
                .best_idx           (slot_best_idx      [s]),
                .best_dist          (slot_best_dist     [s]),

                .cmp_valid          (slot_cmp_valid[s]),
                .cmp_dist           (slot_cmp_dist [s]),
                .cmp_tag            (slot_cmp_tag  [s])
            );
        end
    endgenerate

    // -----------------------------------------------------------------
    // Scheduler
    // -----------------------------------------------------------------
    slot_scheduler #(
        .W             (W),
        .D             (D),
        .IN_WIDTH      (IN_WIDTH),
        .ACC_WIDTH     (ACC_WIDTH),
        .IDX_WIDTH     (IDX_WIDTH),
        .LATENCY       (SLOT_LATENCY),
        .EVICT_INTERVAL(EVICT_INTERVAL)
    ) u_scheduler (
        .clk               (clk),
        .rst_n             (rst_n),

        .in_valid          (in_valid),
        .in_ready          (in_ready),
        .in_arrival_idx    (in_arrival_idx),
        .in_arrival_data   (in_arrival_data),

        .flush             (flush),
        .flush_done        (flush_done),

        .slot_load_valid   (sched_load_valid),
        .slot_load_idx     (sched_load_idx),
        .slot_load_data    (sched_load_data),

        .slot_arrival_valid(sched_arrival_valid),
        .slot_arrival_idx  (sched_arrival_idx),
        .slot_arrival_data (sched_arrival_data),

        .slot_resident_idx (slot_resident_idx),
        .slot_best_idx     (slot_best_idx),
        .slot_best_dist    (slot_best_dist),

        .out_valid         (out_valid),
        .out_ready         (out_ready),
        .out_resident_idx  (out_resident_idx),
        .out_best_idx      (out_best_idx),
        .out_best_dist     (out_best_dist)
    );

endmodule