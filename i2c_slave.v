// -----------------------------------------------------------------------
// i2c_slave.v
// I2C SLAVE RECEIVER, back on the single 50 MHz system clock (clk_50m) -
// NOT clocked by scl. scl/sda are sampled INTO this domain via 2-FF
// synchronizers, and all bit/byte timing (including the ACK assert/
// release) is generated entirely from clk_50m edge-detection.
//
// Why this replaces the SCL-clocked version: running real logic directly
// off SCL required either (a) one always block testing both edges of
// scl - illegal for Quartus synthesis, or (b) splitting FSM/ACK-drive
// across posedge-scl and negedge-scl blocks - which worked in RTL
// simulation but is suspected to hit real clock-network skew between
// the two edges once placed on actual FPGA clock routing (Quartus
// itself flagged these destinations as possibly not using a global/
// regional clock). A single free-running system clock sidesteps all of
// that: one clock, one edge, one clock buffer, trivially analyzable
// timing - the standard way real silicon I2C peripherals are built when
// a fast system clock is available (which it is here: 50 MHz vs a
// 400 kHz bus, 125x margin).
//
// Because rx_data/rx_valid/sda_o now already live in the clk_50m domain,
// byte_cdc.v is no longer needed - the top level wires this module
// straight into sync_fifo.
// -----------------------------------------------------------------------
module i2c_slave #(
    parameter [6:0] OWN_ADDR = 7'h50
)(
    input  wire       clk_50m,
    input  wire       rst_n,

    input  wire       scl,        // raw external I2C clock, sampled (not used as a clock)
    input  wire       sda,        // raw external I2C data, sampled (not used as a clock)

    output reg  [7:0] rx_data,    // completed byte, valid in the clk_50m domain
    output reg        rx_valid,   // 1 clk_50m-cycle pulse when a byte is accepted
    output reg        sda_o       // open-drain drive control: 1 = pull SDA low, 0 = release
);

    // ---- 2-FF synchronizers: bring scl/sda into the clk_50m domain ----
    reg [1:0] scl_ff, sda_ff;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            scl_ff <= 2'b11;
            sda_ff <= 2'b11;
        end else begin
            scl_ff <= {scl_ff[0], scl};
            sda_ff <= {sda_ff[0], sda};
        end
    end

    wire scl_sync = scl_ff[1];
    wire sda_sync = sda_ff[1];

    // ---- edge / condition detection ----
    reg sda_d, scl_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            sda_d <= 1'b1;
            scl_d <= 1'b1;
        end else begin
            sda_d <= sda_sync;
            scl_d <= scl_sync;
        end
    end

    wire scl_rise   =  scl_sync & ~scl_d;
    wire scl_fall   = ~scl_sync &  scl_d;
    wire start_cond =  scl_sync &  scl_d &  sda_d & ~sda_sync; // SCL=1, SDA 1->0
    wire stop_cond  =  scl_sync &  scl_d & ~sda_d &  sda_sync; // SCL=1, SDA 0->1

    // ---- FSM ----
    localparam IDLE           = 3'd0,
               ADDR           = 3'd1,
               ADDR_ACK_SETUP = 3'd2,
               ADDR_ACK_HOLD  = 3'd3,
               DATA           = 3'd4,
               DATA_ACK_SETUP = 3'd5,
               DATA_ACK_HOLD  = 3'd6;

    reg [2:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] shift_reg;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            bit_cnt   <= 3'd0;
            shift_reg <= 8'd0;
            sda_o     <= 1'b0;
            rx_valid  <= 1'b0;
            rx_data   <= 8'd0;
        end else begin
            rx_valid <= 1'b0; // default: one-cycle pulse only

            if (start_cond) begin
                state   <= ADDR;
                bit_cnt <= 3'd0;
                sda_o   <= 1'b0;
            end else if (stop_cond) begin
                state <= IDLE;
                sda_o <= 1'b0;
            end else begin
                case (state)
                    IDLE: begin
                        sda_o <= 1'b0;
                    end

                    // shift in 7 address bits + 1 R/W bit (8 total)
                    ADDR: begin
                        if (scl_rise) begin
                            shift_reg <= {shift_reg[6:0], sda_sync};
                            if (bit_cnt == 3'd7) begin
                                bit_cnt <= 3'd0;
                                state   <= ADDR_ACK_SETUP;
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                            end
                        end
                    end

                    // shift_reg[7:1] = address, shift_reg[0] = R/W
                    ADDR_ACK_SETUP: begin
                        if (scl_fall) begin
                            if ((shift_reg[7:1] == OWN_ADDR) && (shift_reg[0] == 1'b0)) begin
                                sda_o <= 1'b1;   // ACK: our address, write op
                                state <= ADDR_ACK_HOLD;
                            end else begin
                                sda_o <= 1'b0;   // NACK: not us, or a read request
                                state <= IDLE;
                            end
                        end
                    end

                    ADDR_ACK_HOLD: begin
                        if (scl_fall) begin
                            sda_o   <= 1'b0;     // release SDA
                            bit_cnt <= 3'd0;
                            state   <= DATA;
                        end
                    end

                    DATA: begin
                        if (scl_rise) begin
                            shift_reg <= {shift_reg[6:0], sda_sync};
                            if (bit_cnt == 3'd7) begin
                                bit_cnt <= 3'd0;
                                state   <= DATA_ACK_SETUP;
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                            end
                        end
                    end

                    DATA_ACK_SETUP: begin
                        if (scl_fall) begin
                            rx_data  <= shift_reg;
                            sda_o    <= 1'b1;    // ACK: always ACK a data byte, per spec
                            rx_valid <= 1'b1;    // 1-cycle pulse straight to the FIFO
                            state    <= DATA_ACK_HOLD;
                        end
                    end

                    DATA_ACK_HOLD: begin
                        if (scl_fall) begin
                            sda_o <= 1'b0;
                            state <= DATA;       // ready for next byte
                        end
                    end

                    default: state <= IDLE;
                endcase
            end
        end
    end

endmodule
