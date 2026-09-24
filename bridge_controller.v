// -----------------------------------------------------------------------
// bridge_controller.v
// Block 5: Bridge Controller (FSM). Reads bytes out of the FIFO and
// hands them to the UART transmitter, one at a time.
// -----------------------------------------------------------------------
module bridge_controller (
    input  wire       clk_50m,
    input  wire       rst_n,

    input  wire [7:0] fifo_dout,
    input  wire       fifo_empty,
    output reg         fifo_rd_en,

    output reg  [7:0] tx_data,
    output reg         tx_start,
    input  wire        tx_busy
);

    localparam IDLE = 2'd0,
               READ  = 2'd1,   // wait 1 cycle for synchronous FIFO read latency
               LOAD  = 2'd2,
               WAIT  = 2'd3;

    reg [1:0] state;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            fifo_rd_en <= 1'b0;
            tx_start   <= 1'b0;
            tx_data    <= 8'h00;
        end else begin
            fifo_rd_en <= 1'b0; // default: pulse only
            tx_start   <= 1'b0; // default: pulse only

            case (state)
                IDLE: begin
                    if (!fifo_empty && !tx_busy) begin
                        fifo_rd_en <= 1'b1;
                        state      <= READ;
                    end
                end

                READ: begin
                    state <= LOAD; // fifo_dout is now valid
                end

                LOAD: begin
                    tx_data  <= fifo_dout;
                    tx_start <= 1'b1;
                    state    <= WAIT;
                end

                WAIT: begin
                    if (tx_busy)
                        state <= IDLE; // UART has accepted the byte
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
