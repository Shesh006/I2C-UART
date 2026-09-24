// -----------------------------------------------------------------------
// i2c_uart_bridge.v
// Top level: I2C to UART Bridge - Microarchitecture (Detailed), for the
// DE10-Lite (MAX 10).
//
//   clk_in  : 50 MHz -> DE10-Lite pin MAX10_CLK1_50 (PIN_P11)
//   rst_in_n: active-low reset -> DE10-Lite KEY0 (PIN_B8)
//   sda     : open-drain inout -> GPIO pin, needs an EXTERNAL 4.7k
//             pull-up resistor to 3.3V (MAX10 I/O has no built-in one)
//   scl     : input, 400 kHz, drives the I2C-domain logic directly -> GPIO pin
//   uart_tx : output -> GPIO pin, or the onboard USB-UART if available
//
// Pick the actual GPIO pin numbers for sda / scl / uart_tx in Quartus's
// Pin Planner (Assignments -> Pin Planner) based on your wiring.
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

    // ---------------- Block 1: Clock & Reset Generator ----------------
    wire clk_50m;
    wire rst_n;

    clock_reset_gen u_clkrst (
        .clk_in   (clk_in),
        .rst_in_n (rst_in_n),
        .clk_50m  (clk_50m),
        .rst_n    (rst_n)
    );

    // ---------------- Block 2: I2C Slave Receiver (SCL domain) ----------------
    wire [7:0] rx_data;
    wire       rx_valid;
    wire       sda_o;

    i2c_slave #(.OWN_ADDR(I2C_ADDR)) u_i2c_slave (
        .scl      (scl),
        .sda      (sda),
        .rst_n    (rst_n),
        .rx_data  (rx_data),
        .rx_valid (rx_valid),
        .sda_o    (sda_o)
    );

    // open-drain SDA: pull low when driving (ACK), release (Hi-Z) otherwise
    assign sda = sda_o ? 1'b0 : 1'bz;

    // ---------------- Block 3: Byte CDC (I2C -> 50 MHz) ----------------
    wire [7:0] cdc_data;
    wire       cdc_valid;

    byte_cdc u_byte_cdc (
        .rst_n     (rst_n),
        .scl       (scl),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .clk_50m   (clk_50m),
        .cdc_data  (cdc_data),
        .cdc_valid (cdc_valid)
    );

    // ---------------- Block 4: Synchronous FIFO (50 MHz domain) ----------------
    wire [7:0] fifo_dout;
    wire       fifo_empty;
    wire       fifo_full;   // not currently fed back anywhere - see i2c_slave.v note
    wire       fifo_rd_en;

    sync_fifo #(
        .DATA_WIDTH (8),
        .DEPTH      (16),
        .ADDR_WIDTH (4)
    ) u_fifo (
        .clk_50m (clk_50m),
        .rst_n   (rst_n),
        .wr_en   (cdc_valid),
        .din     (cdc_data),
        .full    (fifo_full),
        .rd_en   (fifo_rd_en),
        .dout    (fifo_dout),
        .empty   (fifo_empty)
    );

    // ---------------- Block 5: Bridge Controller (FSM) ----------------
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

    // ---------------- Block 6: UART TX (8-N-1, 115200) ----------------
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
