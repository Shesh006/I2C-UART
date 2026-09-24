// -----------------------------------------------------------------------
// clock_reset_gen.v
// Block 1: Clock & Reset Generator.
// DE10-Lite's onboard clock is already 50 MHz, so no PLL/divider is
// needed - clk_50m is a direct pass-through of clk_in. rst_n is
// asynchronously asserted (immediate) and synchronously released
// (2-stage), the standard safe reset pattern, distributed to every
// other block in the design.
// -----------------------------------------------------------------------
module clock_reset_gen (
    input  wire clk_in,     // 50 MHz external clock
    input  wire rst_in_n,   // external asynchronous active-low reset (e.g. KEY0)

    output wire clk_50m,    // system clock, all 50 MHz-domain blocks
    output reg  rst_n       // synchronized active-low reset, all blocks
);

    assign clk_50m = clk_in;

    reg rst_ff1;

    always @(posedge clk_50m or negedge rst_in_n) begin
        if (!rst_in_n) begin
            rst_ff1 <= 1'b0;
            rst_n   <= 1'b0;
        end else begin
            rst_ff1 <= 1'b1;
            rst_n   <= rst_ff1;   // 2-stage: async assert, sync release
        end
    end

endmodule
