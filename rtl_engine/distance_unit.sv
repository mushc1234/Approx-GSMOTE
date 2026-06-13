// distance_unit_fully_pipelined.sv
// =====================================================================
// Iterative squared Euclidean distance with 3-stage internal pipeline.
//
// Architecture:
//   - Serializer: Captures wide inputs and shifts them right.
//   - Stage 1 (Sub): Subtracts active lanes and registers differences.
//   - Stage 2 (MAC): Squares and accumulates per lane. Registers the 
//                    final per-lane totals on the last chunk.
//   - Stage 3 (Add): Sums the parallel lane totals into the final 
//                    distance output.
// =====================================================================

module distance_unit #(
    parameter int D         = 8,
    parameter int LANES     = 4,      
    parameter int IN_WIDTH  = 16,
    parameter int ACC_WIDTH = 40,
    parameter int TAG_WIDTH = 32
) (
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic                  in_valid,
    input  logic [D*IN_WIDTH-1:0] in_a,
    input  logic [D*IN_WIDTH-1:0] in_b,
    input  logic [TAG_WIDTH-1:0]  in_tag,

    output logic                  out_valid,
    output logic [ACC_WIDTH-1:0]  out_dist,
    output logic [TAG_WIDTH-1:0]  out_tag
);

    localparam int CYCLES = D / LANES;
    localparam int COUNTER_WIDTH = $clog2(CYCLES + 1);

    // -----------------------------------------------------------------
    // FSM / Serializer
    // -----------------------------------------------------------------
    logic [D*IN_WIDTH-1:0]    shift_a, shift_b;
    logic [COUNTER_WIDTH-1:0] count;
    logic                     active;
    logic [TAG_WIDTH-1:0]     tag_reg;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            active   <= 1'b0;
        end else begin
            if (in_valid) begin
                shift_a  <= in_a;
                shift_b  <= in_b;
                tag_reg  <= in_tag;
                count    <= CYCLES[COUNTER_WIDTH-1:0];
                active   <= 1'b1; 
            end else if (active) begin
                shift_a <= shift_a >> (LANES * IN_WIDTH);
                shift_b <= shift_b >> (LANES * IN_WIDTH);
                count   <= count - 1'b1;
                
                if (count == 1) begin
                    active   <= 1'b0; 
                end
            end
        end
    end

    // -----------------------------------------------------------------
    // Pipeline Stage 1: Subtraction
    // -----------------------------------------------------------------
    logic                     s1_valid;
    logic                     s1_last;
    logic signed [IN_WIDTH:0] s1_diff [LANES];
    logic [TAG_WIDTH-1:0]     s1_tag;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
            s1_last  <= 1'b0;
        end else begin
            s1_valid <= active;
            s1_last  <= active && (count == 1);
            s1_tag   <= tag_reg;
            
            for (int i = 0; i < LANES; i++) begin
                s1_diff[i] <= $signed(shift_a[(i+1)*IN_WIDTH-1 -: IN_WIDTH])
                            - $signed(shift_b[(i+1)*IN_WIDTH-1 -: IN_WIDTH]);
            end
        end
    end

    // -----------------------------------------------------------------
    // Pipeline Stage 2: Parallel MACs 
    // -----------------------------------------------------------------
    logic [ACC_WIDTH-1:0] sq [LANES];
    logic [ACC_WIDTH-1:0] acc [LANES];

    logic                 s2_valid;
    logic [TAG_WIDTH-1:0] s2_tag;
    logic [ACC_WIDTH-1:0] s2_lane_tot [LANES];

    always_comb begin
        for (int i = 0; i < LANES; i++) begin
            sq[i] = ACC_WIDTH'(s1_diff[i] * s1_diff[i]);
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s2_valid <= 1'b0;
            s2_tag   <= '0;
            for (int i = 0; i < LANES; i++) begin
                acc[i]         <= '0;
                s2_lane_tot[i] <= '0;
            end
        end else begin
            s2_valid <= 1'b0; 
            
            if (s1_valid) begin
                if (s1_last) begin
                    // Final chunk: Register the lane totals and pass to Stage 3
                    s2_valid <= 1'b1;
                    s2_tag   <= s1_tag;
                    for (int i = 0; i < LANES; i++) begin
                        s2_lane_tot[i] <= acc[i] + sq[i];
                        acc[i]         <= '0; // Clear for next vector
                    end
                end else begin
                    // Intermediate chunks: Keep accumulating locally
                    for (int i = 0; i < LANES; i++) begin
                        acc[i] <= acc[i] + sq[i];
                    end
                end
            end
        end
    end

    // -----------------------------------------------------------------
    // Pipeline Stage 3: Final Adder Tree
    // -----------------------------------------------------------------
    logic [ACC_WIDTH-1:0] final_tree_sum;

    always_comb begin
        final_tree_sum = '0;
        for (int i = 0; i < LANES; i++) begin
            final_tree_sum = final_tree_sum + s2_lane_tot[i];
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_dist  <= '0;
            out_tag   <= '0;
        end else begin
            // Default drop
            out_valid <= 1'b0;
            
            if (s2_valid) begin
                out_valid <= 1'b1;
                out_dist  <= final_tree_sum;
                out_tag   <= s2_tag;
            end
        end
    end

endmodule