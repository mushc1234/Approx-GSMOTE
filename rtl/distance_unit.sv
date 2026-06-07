// distance_unit_folded.sv
// =====================================================================
// Iterative (MAC-style) squared Euclidean distance between two D-dim vectors.
//
// Architecture:
//   - Uses a folded datapath, processing `LANES` dimensions per cycle.
//   - Captures wide inputs into shift registers and shifts them right 
//     each cycle to feed the MAC units.
//   - Latency = (D / LANES) cycles + 1 overhead cycle.
//   - Requires handshaking (`in_ready` / `in_valid`) since it can no
//     longer accept a new vector every clock cycle.
// =====================================================================

module distance_unit #(
    parameter int D         = 8,
    parameter int LANES     = 4,      // Number of parallel squarers (MACs) per cycle
    parameter int IN_WIDTH  = 16,
    parameter int ACC_WIDTH = 40,
    parameter int TAG_WIDTH = 32
) (
    input  logic                  clk,
    input  logic                  rst_n,

    // Input Interface (now requires handshaking/backpressure)
    input  logic                  in_valid,
    input  logic [D*IN_WIDTH-1:0] in_a,
    input  logic [D*IN_WIDTH-1:0] in_b,
    input  logic [TAG_WIDTH-1:0]  in_tag,

    // Output Interface
    output logic                  out_valid,
    output logic [ACC_WIDTH-1:0]  out_dist,
    output logic [TAG_WIDTH-1:0]  out_tag
);

    // Ensure D is cleanly divisible by LANES
    // synthesis translate_off
    initial begin
        if (D % LANES != 0) $fatal(1, "D must be a multiple of LANES");
    end
    // synthesis translate_on

    localparam int CYCLES = D / LANES;
    localparam int COUNTER_WIDTH = $clog2(CYCLES + 1);

    // -----------------------------------------------------------------
    // FSM and Registers
    // -----------------------------------------------------------------
    typedef enum logic { IDLE, COMPUTE } state_t;
    state_t state;

    logic [D*IN_WIDTH-1:0]    shift_a;
    logic [D*IN_WIDTH-1:0]    shift_b;
    logic [TAG_WIDTH-1:0]     tag_reg;
    logic [ACC_WIDTH-1:0]     acc;
    logic [COUNTER_WIDTH-1:0] count;

    // -----------------------------------------------------------------
    // Combinational Datapath (The "MAC" Chunk)
    // -----------------------------------------------------------------
    logic signed [IN_WIDTH:0] diff [LANES];
    logic [ACC_WIDTH-1:0]     lane_sum;

    always_comb begin
        lane_sum = '0;
        for (int i = 0; i < LANES; i++) begin
            // 1. Subtract the bottom-most lanes currently in the shift register
            diff[i] = $signed(shift_a[(i+1)*IN_WIDTH-1 -: IN_WIDTH])
                    - $signed(shift_b[(i+1)*IN_WIDTH-1 -: IN_WIDTH]);
            
            // 2. Square and sum combinationally for this specific clock cycle
            lane_sum = lane_sum + ACC_WIDTH'(diff[i] * diff[i]);
        end
    end

    // -----------------------------------------------------------------
    // Synchronous Logic & Control
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state     <= IDLE;
            out_valid <= 1'b0;
            out_dist  <= '0;
            out_tag   <= '0;
            count     <= '0;
            acc       <= '0;
        end else begin
            // Default pulse assertion
            out_valid <= 1'b0;

            case (state)
                IDLE: begin
                    if (in_valid) begin
                        shift_a  <= in_a;          // Snapshot inputs
                        shift_b  <= in_b;
                        tag_reg  <= in_tag;
                        acc      <= '0;            // Clear the accumulator
                        count    <= CYCLES[COUNTER_WIDTH-1:0]; 
                        state    <= COMPUTE;
                    end
                end

                COMPUTE: begin
                    // Accumulate the sum from the current lanes
                    acc <= acc + lane_sum;

                    // Shift the registers right to expose the next chunk of data
                    shift_a <= shift_a >> (LANES * IN_WIDTH);
                    shift_b <= shift_b >> (LANES * IN_WIDTH);

                    count <= count - 1'b1;

                    // If we are on the last cycle of computation
                    if (count == 1) begin
                        out_valid <= 1'b1;
                        // Output the running accumulator + the final chunk's sum directly 
                        // to save an extra clock cycle of latency
                        out_dist  <= acc + lane_sum; 
                        out_tag   <= tag_reg;

                        state     <= IDLE;
                    end
                end
            endcase
        end
    end

endmodule