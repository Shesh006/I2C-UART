// -----------------------------------------------------------------------
// i2c_uart_bridge.v
// Top level: I2C SLAVE (clk_50m domain) -> UART TX bridge, DE10-Lite.
//
// byte_cdc is REMOVED: since i2c_slave now runs entirely on clk_50m,
// its rx_data/rx_valid outputs are already in the right clock domain -
// they wire straight into sync_fifo, no crossing needed.
//
//   clk_in  : 50 MHz -> DE10-Lite pin MAX10_CLK1_50 (PIN_P11)
//   rst_in_n: active-low reset -> DE10-Lite KEY0 (PIN_B8)
//   sda     : open-drain inout -> GPIO pin, needs an EXTERNAL 4.7k
//             pull-up resistor to 3.3V
//   scl     : input, 400 kHz -> GPIO pin (sampled, not used as a clock)
//   uart_tx : output -> GPIO pin, or an onboard USB-UART if available
// -----------------------------------------------------------------------
module i2c_uart_bridge #(
    parameter [6:0] I2C_ADDR = 7'h50
)(
    input  wire clk_in,
    input  wire rst_in_n,

    inout  wire sda,
    input  wire scl,

    output wire uart_tx
);

    // ---------------- Clock & Reset Generator ----------------
    wire clk_50m;
    wire rst_n;

    clock_reset_gen u_clkrst (
        .clk_in   (clk_in),
        .rst_in_n (rst_in_n),
        .clk_50m  (clk_50m),
        .rst_n    (rst_n)
    );

    // ---------------- I2C Slave Receiver (clk_50m domain) ----------------
    wire [7:0] rx_data;
    wire       rx_valid;
    wire       sda_o;

    i2c_slave #(.OWN_ADDR(I2C_ADDR)) u_i2c_slave (
        .clk_50m  (clk_50m),
        .rst_n    (rst_n),
        .scl      (scl),
        .sda      (sda),
        .rx_data  (rx_data),
        .rx_valid (rx_valid),
        .sda_o    (sda_o)
    );

    // open-drain SDA: pull low when driving (ACK), release (Hi-Z) otherwise
    assign sda = sda_o ? 1'b0 : 1'bz;

    // ---------------- Synchronous FIFO ----------------
    wire [7:0] fifo_dout;
    wire       fifo_empty;
    wire       fifo_full;
    wire       fifo_rd_en;

    sync_fifo #(
        .DATA_WIDTH (8),
        .DEPTH      (16),
        .ADDR_WIDTH (4)
    ) u_fifo (
        .clk_50m (clk_50m),
        .rst_n   (rst_n),
        .wr_en   (rx_valid),
        .din     (rx_data),
        .full    (fifo_full),
        .rd_en   (fifo_rd_en),
        .dout    (fifo_dout),
        .empty   (fifo_empty)
    );

    // ---------------- Bridge Controller (FSM) ----------------
    wire [7:0] tx_data;
    wire       tx_start;
    wire       tx_busy;

    bridge_controller u_bridge (
        .clk_50m    (clk_50m),
        .rst_n      (rst_n),
        .fifo_dout  (fifo_dout),
        .fifo_empty (fifo_empty),
        .fifo_rd_en (fifo_rd_en),
        .tx_data    (tx_data),
        .tx_start   (tx_start),
        .tx_busy    (tx_busy)
    );

    // ---------------- UART TX (8-N-1, 115200) ----------------
    uart_tx #(
        .CLK_FREQ  (50_000_000),
        .BAUD_RATE (115200)
    ) u_uart_tx (
        .clk_50m  (clk_50m),
        .rst_n    (rst_n),
        .tx_data  (tx_data),
        .tx_start (tx_start),
        .tx_busy  (tx_busy),
        .uart_tx  (uart_tx)
    );

endmodule
