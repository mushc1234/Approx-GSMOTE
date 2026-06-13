// record_splitter.sv
// =====================================================================
// Reassembles records from a beat stream.
//
// Record layout on the bus (interleaved DDR layout):
//   beat 0:  {16'b pad, 16'b id}      (id in low 16 bits)
//   beat 1:  data words [31:0]
//   beat 2:  data words [63:32]
//   beat 3:  data words [95:64]
//   beat 4:  data words [127:96]
//   ...
//   total beats per record = 1 + ceil(D*IN_WIDTH / AXI_DATA_WIDTH)
//
// Output is a single wide bundle per record, presented with a
// downstream ready/valid handshake.
// =====================================================================

module record_splitter #(
    parameter int D              = 8,
    parameter int IN_WIDTH       = 16,
    parameter int IDX_WIDTH      = 16,
    parameter int AXI_DATA_WIDTH = 32
) (
    input  logic                          clk,
    input  logic                          rst_n,

    // Beat stream from AXI master
    input  logic                          in_valid,
    output logic                          in_ready,
    input  logic [AXI_DATA_WIDTH-1:0]     in_data,
    input  logic                          in_last,

    // Assembled record to downstream
    output logic                          out_valid,
    input  logic                          out_ready,
    output logic [IDX_WIDTH-1:0]          out_id,
    output logic [D*IN_WIDTH-1:0]         out_data
);

    localparam int DATA_BITS  = D * IN_WIDTH;
    localparam int DATA_BEATS = (DATA_BITS + AXI_DATA_WIDTH - 1) / AXI_DATA_WIDTH;
    localparam int BEATS      = 1 + DATA_BEATS;
    localparam int BEAT_W     = $clog2(BEATS + 1);

    // Padded data buffer (rounded up to whole AXI words)
    localparam int DATA_BUF_BITS = DATA_BEATS * AXI_DATA_WIDTH;

    logic [DATA_BUF_BITS-1:0]   data_buf_r;
    logic [IDX_WIDTH-1:0]       id_r;
    logic [BEAT_W-1:0]          beat_r;
    logic                       have_record_r;

    // Hold record until consumed
    assign out_valid = have_record_r;
    assign out_id    = id_r;
    assign out_data  = data_buf_r[D*IN_WIDTH-1:0];

    // Accept beats unless we're sitting on an unconsumed record
    assign in_ready  = !have_record_r;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            data_buf_r    <= '0;
            id_r          <= '0;
            beat_r        <= '0;
            have_record_r <= 1'b0;
        end else begin
            // Downstream consumes
            if (have_record_r && out_ready) begin
                have_record_r <= 1'b0;
                beat_r        <= '0;
            end

            // Accept beat
            if (in_valid && in_ready) begin
                if (beat_r == 0) begin
                    id_r <= in_data[IDX_WIDTH-1:0];
                end else begin
                    // Pack into the appropriate AXI-word slot of data_buf
                    // Slot index = beat_r - 1
                    for (int slot = 0; slot < DATA_BEATS; slot++) begin
                        if ((BEAT_W'(slot) + BEAT_W'(1)) == beat_r) begin
                            data_buf_r[(slot+1)*AXI_DATA_WIDTH-1 -: AXI_DATA_WIDTH]
                                <= in_data;
                        end
                    end
                end

                if (in_last) begin
                    have_record_r <= 1'b1;
                end else begin
                    beat_r <= beat_r + 1'b1;
                end
            end
        end
    end

endmodule
