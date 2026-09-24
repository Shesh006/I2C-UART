// -----------------------------------------------------------------------
// uart_tx.v
// Block 6: UART TX (8-N-1, 115200 baud). Baud-rate generator + TX FSM +
// TX shift register + bit counter + output driver, all as drawn.
// -----------------------------------------------------------------------
module uart_tx #(
    parameter integer CLK_FREQ  = 50_000_000,
    parameter integer BAUD_RATE = 115200
)(
    input  wire       clk_50m,
    input  wire       rst_n,

    input  wire [7:0] tx_data,
    input  wire       tx_start,   // pulse for 1 clk to begin a frame
    output reg         tx_busy,

    output reg         uart_tx    // serial line, idles high
);

    localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE; // ~434 @ 50MHz/115200

    // ---- Baud Rate Generator ----
    reg [15:0] baud_cnt;
    wire baud_tick = (baud_cnt == BAUD_DIV - 1);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt <= 16'd0;
        end else if (tx_busy) begin
            baud_cnt <= baud_tick ? 16'd0 : baud_cnt + 1'b1;
        end else begin
            baud_cnt <= 16'd0;
        end
    end

    // ---- TX FSM + 10-bit shift register + bit counter ----
    localparam IDLE  = 2'd0,
               START = 2'd1,
               DATA  = 2'd2,
               STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] bit_idx;   // 0-9 for the full 10-bit frame (start+8 data+stop)
    reg [7:0] shift_reg;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            uart_tx   <= 1'b1;
            tx_busy   <= 1'b0;
            bit_idx   <= 4'd0;
            shift_reg <= 8'd0;
        end else begin
            case (state)
                IDLE: begin
                    uart_tx <= 1'b1;
                    if (tx_start) begin
                        shift_reg <= tx_data;
                        tx_busy   <= 1'b1;
                        state     <= START;
                    end
                end

                START: begin
                    uart_tx <= 1'b0;               // start bit
                    if (baud_tick) begin
                        state   <= DATA;
                        bit_idx <= 4'd0;
                    end
                end

                DATA: begin
                    uart_tx <= shift_reg[0];        // LSB first
                    if (baud_tick) begin
                        shift_reg <= shift_reg >> 1;
                        if (bit_idx == 4'd7)
                            state <= STOP;
                        else
                            bit_idx <= bit_idx + 1'b1;
                    end
                end

                STOP: begin
                    uart_tx <= 1'b1;                // stop bit
                    if (baud_tick) begin
                        tx_busy <= 1'b0;
                        state   <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
