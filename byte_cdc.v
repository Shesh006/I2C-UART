// -----------------------------------------------------------------------
// byte_cdc.v
// Block 3: Byte CDC (I2C -> 50 MHz).
// Converts the 1-cycle rx_valid pulse (in the slow, irregular SCL
// domain) into an internal toggle - a single bit is far easier to
// synchronize reliably than trying to catch a short pulse directly with
// a fast-domain synchronizer. The toggle is then synchronized into the
// 50 MHz domain (3-stage: 2-FF sync + 1 extra stage for edge detect),
// and cdc_valid pulses for exactly one clk_50m cycle when a transition
// is detected. cdc_data is captured from data_reg at that same moment -
// data_reg has been stable in the SCL domain since the toggle changed,
// long before the (much faster) 50 MHz side notices it, so no per-bit
// tearing is possible - unlike naive independent 2-FF sync per bit.
// -----------------------------------------------------------------------
module byte_cdc (
    input  wire       rst_n,
    input  wire       scl,          // I2C-domain clock (write side, 400 kHz)
    input  wire [7:0] rx_data,
    input  wire       rx_valid,     // 1 scl-cycle pulse

    input  wire       clk_50m,      // system-domain clock (read side, 50 MHz)
    output reg  [7:0] cdc_data,
    output reg        cdc_valid     // 1 clk_50m-cycle pulse
);

    // ---- write side: I2C (scl) domain ----
    // "Toggle flip-flop (I2C clk)" + "Data Register (8-bit)"
    reg [7:0] data_reg;
    reg       toggle;

    always @(posedge scl or negedge rst_n) begin
        if (!rst_n) begin
            data_reg <= 8'h00;
            toggle   <= 1'b0;
        end else if (rx_valid) begin
            data_reg <= rx_data;
            toggle   <= ~toggle;
        end
    end

    // ---- read side: 50 MHz domain ----
    // "2-FF sync (50 MHz)" + "Edge Detect"
    reg tog_ff1, tog_ff2, tog_ff3;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tog_ff1   <= 1'b0;
            tog_ff2   <= 1'b0;
            tog_ff3   <= 1'b0;
            cdc_data  <= 8'h00;
            cdc_valid <= 1'b0;
        end else begin
            tog_ff1 <= toggle;
            tog_ff2 <= tog_ff1;
            tog_ff3 <= tog_ff2;

            cdc_valid <= 1'b0;
            if (tog_ff2 != tog_ff3) begin
                cdc_data  <= data_reg;
                cdc_valid <= 1'b1;
            end
        end
    end

endmodule
