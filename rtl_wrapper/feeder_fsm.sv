// feeder_fsm.sv
// =====================================================================
// Takes assembled records from the splitter and drives the engine's
// in_arrival_* port with proper ready/valid handshake. Counts the
// records pushed so the run FSM knows when to assert flush.
//
// Reset condition: at the start of each run, the caller drives
// reset_count high for one cycle to zero records_pushed.
// =====================================================================

module feeder_fsm #(
    parameter int D         = 8,
    parameter int IN_WIDTH  = 16,
    parameter int IDX_WIDTH = 16
) (
    input  logic                       clk,
    input  logic                       rst_n,

    // Control
    input  logic                       enable,
    input  logic                       reset_count,

    // Input: assembled record from splitter
    input  logic                       rec_valid,
    output logic                       rec_ready,
    input  logic [IDX_WIDTH-1:0]       rec_id,
    input  logic [D*IN_WIDTH-1:0]      rec_data,

    // Output: engine arrival port
    output logic                       eng_in_valid,
    input  logic                       eng_in_ready,
    output logic [IDX_WIDTH-1:0]       eng_in_arrival_idx,
    output logic [D*IN_WIDTH-1:0]      eng_in_arrival_data,

    // Status
    output logic [31:0]                records_pushed
);

    // Pass-through: handshake gated by `enable`
    assign eng_in_valid        = enable && rec_valid;
    assign eng_in_arrival_idx  = rec_id;
    assign eng_in_arrival_data = rec_data;
    assign rec_ready           = enable && eng_in_ready;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            records_pushed <= '0;
        end else if (reset_count) begin
            records_pushed <= '0;
        end else if (eng_in_valid && eng_in_ready) begin
            records_pushed <= records_pushed + 1'b1;
        end
    end

endmodule
