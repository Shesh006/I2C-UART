// -----------------------------------------------------------------------
// sync_fifo.v
// Block 4: Synchronous FIFO (8-bit x 16), single clock domain (clk_50m).
// -----------------------------------------------------------------------
module sync_fifo #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH      = 16,
    parameter ADDR_WIDTH = 4   // must satisfy 2**ADDR_WIDTH >= DEPTH
)(
    input  wire                   clk_50m,
    input  wire                   rst_n,

    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  din,
    output wire                   full,

    input  wire                   rd_en,
    output reg  [DATA_WIDTH-1:0]  dout,
    output wire                   empty
);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr, rd_ptr;
    reg [ADDR_WIDTH:0]   count; // extra bit so it can represent DEPTH itself

    assign full  = (count == DEPTH);
    assign empty = (count == 0);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
            dout   <= 0;
        end else begin
            case ({wr_en && !full, rd_en && !empty})
                2'b10: begin // write only
                    mem[wr_ptr] <= din;
                    wr_ptr      <= wr_ptr + 1'b1;
                    count       <= count + 1'b1;
                end
                2'b01: begin // read only
                    dout   <= mem[rd_ptr];
                    rd_ptr <= rd_ptr + 1'b1;
                    count  <= count - 1'b1;
                end
                2'b11: begin // simultaneous read + write, count unchanged
                    mem[wr_ptr] <= din;
                    dout        <= mem[rd_ptr];
                    wr_ptr      <= wr_ptr + 1'b1;
                    rd_ptr      <= rd_ptr + 1'b1;
                end
                default: ; // no operation
            endcase
        end
    end

endmodule
