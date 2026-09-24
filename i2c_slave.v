// -----------------------------------------------------------------------
// i2c_slave.v
// Block 2: I2C Slave Receiver.
//
// Split across TWO single-edge always blocks (Quartus rejects testing
// both edges of the same signal in ONE always block, but is completely
// fine with two separate single-edge blocks on posedge/negedge of the
// same clock - the same technique DDR interfaces use):
//
//   - Main FSM (posedge scl): bit counting, address/data capture,
//     computes ack_pending (whether this byte should be ACKed) at the
//     edge that completes the 8th bit.
//   - sda_o driver (negedge scl, sole owner of sda_o): asserts ACK right
//     as the low phase BEFORE the ack bit begins (so it's rock solid by
//     the time the master samples it on the next rising edge), and
//     releases it right as the low phase AFTER the ack bit begins (so
//     it's out of the way before the master sets up its next bit) - NOT
//     on the same edge the master samples ACK on, which is the bug this
//     fixes: releasing on that same edge means the release is fully
//     visible (nonblocking, resolved well before any later sample time)
//     by the time the master actually checks it.
//
// START/STOP detection: SDA transitions while SCL is stable can't be
// seen by an scl-edge-only process directly, so two tiny registers below
// are clocked by SDA's OWN edges to catch them; the main FSM re-samples
// those flags synchronously on its own posedge scl (same-domain edge
// detection - not a bus synchronizer).
//
// Per the architecture as specified, this block always ACKs a matching
// address + any data byte - no fifo_full/backpressure input, matching
// the signal tables given (neither this block nor the FIFO define one).
// -----------------------------------------------------------------------
module i2c_slave #(
    parameter [6:0] OWN_ADDR = 7'h50
)(
    input  wire       scl,        // I2C clock (400 kHz) - drives this module directly
    input  wire       sda,        // I2C data, sampled directly
    input  wire       rst_n,

    output reg  [7:0] rx_data,    // completed byte, held stable in the SCL domain
    output reg        rx_valid,   // 1 scl-cycle pulse when a byte is accepted
    output reg        sda_o       // open-drain drive control: 1 = pull SDA low, 0 = release
);

    // ---- async START/STOP flags, clocked by SDA's own edges ----
    reg start_raw, stop_raw;

    always @(negedge sda or negedge rst_n) begin
        if (!rst_n)
            start_raw <= 1'b0;
        else
            start_raw <= scl;    // SDA falls while SCL high -> START condition
    end

    always @(posedge sda or negedge rst_n) begin
        if (!rst_n)
            stop_raw <= 1'b0;
        else
            stop_raw <= scl;     // SDA rises while SCL high -> STOP condition
    end

    // ---- FSM state ----
    localparam IDLE     = 3'd0,
               ADDR      = 3'd1,
               ADDR_ACK  = 3'd2,
               DATA      = 3'd3,
               DATA_ACK  = 3'd4;

    reg [2:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] shift_reg;
    reg       start_raw_d, stop_raw_d;   // same-domain edge detectors
    reg       ack_pending;               // set at the byte-completing edge; read by the negedge block

    // ================== Main FSM: posedge scl only ==================
    always @(posedge scl or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            bit_cnt     <= 3'd0;
            shift_reg   <= 8'd0;
            rx_data     <= 8'd0;
            rx_valid    <= 1'b0;
            start_raw_d <= 1'b0;
            stop_raw_d  <= 1'b0;
            ack_pending <= 1'b0;
        end else begin
            rx_valid    <= 1'b0;      // default: one-cycle pulse only
            start_raw_d <= start_raw;
            stop_raw_d  <= stop_raw;

            if (start_raw && !start_raw_d) begin
                // fresh START - this same rising edge IS address bit 1
                shift_reg <= {7'b0, sda};
                bit_cnt   <= 3'd1;
                state     <= ADDR;
            end else if (stop_raw && !stop_raw_d) begin
                state <= IDLE;
            end else begin
                case (state)
                    IDLE: begin
                        // nothing to do
                    end

                    // shift in 7 address bits + 1 R/W bit (8 total)
                    ADDR: begin
                        if (bit_cnt == 3'd7) begin
                            // this edge completes the address+R/W byte:
                            // shift_reg[6:0] = 7 address bits already
                            // captured over previous edges, sda (fresh,
                            // this edge) = the R/W bit
                            if ((shift_reg[6:0] == OWN_ADDR) && (sda == 1'b0)) begin
                                ack_pending <= 1'b1;   // ACK: our address, write op
                                state       <= ADDR_ACK;
                            end else begin
                                ack_pending <= 1'b0;   // NACK: not us, or a read request
                                state       <= IDLE;
                            end
                            bit_cnt <= 3'd0;
                        end else begin
                            shift_reg <= {shift_reg[6:0], sda};
                            bit_cnt   <= bit_cnt + 1'b1;
                        end
                    end

                    ADDR_ACK: begin
                        bit_cnt <= 3'd0;
                        state   <= DATA;
                    end

                    DATA: begin
                        if (bit_cnt == 3'd7) begin
                            // this edge completes the data byte:
                            // {shift_reg[6:0], sda}
                            rx_data     <= {shift_reg[6:0], sda};
                            rx_valid    <= 1'b1;      // 1-cycle pulse to byte_cdc
                            ack_pending <= 1'b1;      // always ACK a data byte, per spec
                            state       <= DATA_ACK;
                            bit_cnt     <= 3'd0;
                        end else begin
                            shift_reg <= {shift_reg[6:0], sda};
                            bit_cnt   <= bit_cnt + 1'b1;
                        end
                    end

                    DATA_ACK: begin
                        state <= DATA;   // ready for next byte
                    end

                    default: state <= IDLE;
                endcase
            end
        end
    end

    // ========== sda_o driver: negedge scl only, sole owner ==========
    // Asserts ACK exactly as the low phase before the ack bit begins;
    // releases exactly as the low phase after the ack bit begins.
    always @(negedge scl or negedge rst_n) begin
        if (!rst_n) begin
            sda_o <= 1'b0;
        end else begin
            if (state == ADDR_ACK || state == DATA_ACK)
                sda_o <= ack_pending;
            else
                sda_o <= 1'b0;
        end
    end

endmodule
