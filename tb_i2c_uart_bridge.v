// -----------------------------------------------------------------------
// tb_i2c_uart_bridge.v
// Fully automatic testbench for i2c_uart_bridge (clk_in/rst_in_n/sda/scl/
// uart_tx port names). FIFO is 8x64 with fifo_full-aware ACK/NACK in
// i2c_slave. Compile, then:
//     vsim work.tb_i2c_uart_bridge
//     run -all
// Drives 9 test cases itself and stops on its own - no typing required.
//
// Full compile + run, from a clean folder:
//   vlib work
//   vlog clock_reset_gen.v i2c_slave.v sync_fifo.v \
//        bridge_controller.v uart_tx.v i2c_uart_bridge.v tb_i2c_uart_bridge.v
//   vsim work.tb_i2c_uart_bridge
//   run -all
// -----------------------------------------------------------------------
`timescale 1ns/1ps

module tb_i2c_uart_bridge;

    // ---------------- parameters ----------------
    parameter CLK_PERIOD      = 20;     // 50 MHz -> 20 ns
    parameter I2C_HALF_PERIOD = 1250;   // 400 kHz -> 2.5 us period, 1.25 us half
    parameter [6:0] OWN_ADDR   = 7'h50;
    parameter [6:0] WRONG_ADDR = 7'h51;

    // UART bit period must match the DUT's own baud generator exactly:
    // 50_000_000 / 115200 = 434 (integer division) system clocks per bit.
    parameter UART_BIT_TIME = 434 * CLK_PERIOD; // 8680 ns

    // ---------------- DUT signals ----------------
    reg  clk_in;
    reg  rst_in_n;
    reg  scl;
    reg  sda_drive_low;   // testbench's own SDA drive control
    tri1 sda;             // idles high, either side (or DUT) can pull low
    wire uart_tx;

    assign sda = sda_drive_low ? 1'b0 : 1'bz;

    // No parameter override here on purpose: the gate-level netlist
    // (I2C-UART.vo) has I2C_ADDR resolved and stripped by Quartus's EDA
    // netlist writer, so vopt can't find it to override. The top-level
    // default (7'h50) already matches OWN_ADDR below, so this works
    // identically for both RTL and gate-level simulation.
    i2c_uart_bridge dut (
        .clk_in   (clk_in),
        .rst_in_n (rst_in_n),
        .sda      (sda),
        .scl      (scl),
        .uart_tx  (uart_tx)
    );

    // ---------------- clock ----------------
    initial begin
        clk_in = 1'b0;
        forever #(CLK_PERIOD/2) clk_in = ~clk_in;
    end

    // ---------------- reset ----------------
    initial begin
        rst_in_n      = 1'b0;
        scl           = 1'b1;
        sda_drive_low = 1'b0;
        #500;             // generous margin for clock_reset_gen's 2-stage sync
        rst_in_n = 1'b1;
    end

    // ---------------- waveform dump ----------------
    initial begin
        $dumpfile("tb_i2c_uart_bridge.vcd");
        $dumpvars(0, tb_i2c_uart_bridge);
    end

    // ---------------- global watchdog ----------------
    initial begin
        #20_000_000; // 20 ms - generous upper bound for all tests below,
                     // including test 9's 100-byte burst plus waiting for
                     // every ACKed byte to drain out over UART afterward
                     // (that drain alone can take up to ~10 ms)
        $display("[%0t] GLOBAL TIMEOUT - simulation did not finish in time", $time);
        $stop;
    end

    // =======================================================================
    // I2C MASTER (bit-banged, 400 kHz)
    // =======================================================================

    task i2c_start;
        begin
            sda_drive_low = 1'b0;   // SDA released -> high
            scl           = 1'b1;
            #I2C_HALF_PERIOD;
            sda_drive_low = 1'b1;   // SDA falls while SCL high -> START
            #I2C_HALF_PERIOD;
            scl = 1'b0;             // bring SCL low, ready for first bit
        end
    endtask

    task i2c_stop;
        begin
            scl           = 1'b0;
            sda_drive_low = 1'b1;   // SDA low
            #I2C_HALF_PERIOD;
            scl = 1'b1;             // SCL high, SDA still low
            #I2C_HALF_PERIOD;
            sda_drive_low = 1'b0;   // SDA released -> rises while SCL high -> STOP
            #I2C_HALF_PERIOD;
        end
    endtask

    task i2c_write_bit;
        input b;
        begin
            scl           = 1'b0;   // change data while SCL low
            sda_drive_low = ~b;     // b=1 -> release (high), b=0 -> drive low
            #I2C_HALF_PERIOD;
            scl = 1'b1;             // rising edge - DUT samples here
            #I2C_HALF_PERIOD;
        end
    endtask

    // reads the ACK/NACK bit the DUT drives back
    task i2c_read_ack;
        output ack;
        begin
            scl           = 1'b0;   // falling edge tells DUT to drive its ACK
            sda_drive_low = 1'b0;   // release SDA so the DUT can drive it
            #I2C_HALF_PERIOD;
            scl = 1'b1;
            #10;                    // small settle margin
            ack = ~sda;             // ACK = SDA held low by DUT
            #(I2C_HALF_PERIOD-10);
        end
    endtask

    task i2c_write_byte;
        input [7:0] data;
        output ack;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1)
                i2c_write_bit(data[i]);
            i2c_read_ack(ack);
        end
    endtask

    // single-byte transaction: START, ADDR+W, 1 data byte, STOP
    task i2c_write_transaction_1;
        input [6:0] addr;
        input [7:0] data;
        output addr_ack;
        output data_ack;
        begin
            i2c_start;
            i2c_write_byte({addr, 1'b0}, addr_ack); // R/W = 0 (write)
            if (addr_ack)
                i2c_write_byte(data, data_ack);
            else
                data_ack = 1'b0;
            i2c_stop;
        end
    endtask

    // 5-byte burst transaction: START, ADDR+W, 5 data bytes, STOP
    task i2c_write_transaction_5;
        input [6:0] addr;
        input [7:0] d0, d1, d2, d3, d4;
        output addr_ack;
        output all_ack;
        reg ack;
        begin
            i2c_start;
            i2c_write_byte({addr, 1'b0}, addr_ack);
            all_ack = addr_ack;
            if (addr_ack) begin
                i2c_write_byte(d0, ack); all_ack = all_ack & ack;
                i2c_write_byte(d1, ack); all_ack = all_ack & ack;
                i2c_write_byte(d2, ack); all_ack = all_ack & ack;
                i2c_write_byte(d3, ack); all_ack = all_ack & ack;
                i2c_write_byte(d4, ack); all_ack = all_ack & ack;
            end
            i2c_stop;
        end
    endtask

    // N-byte burst, same value repeated: START, ADDR+W, n data bytes, STOP.
    // Keeps sending even if a data byte is NACKed (real hardware would
    // normally stop on a NACK, but the DUT's FSM accepts further bytes
    // either way, so this lets us count exactly how many of n got ACKed
    // - the point of the FIFO-overflow test below).
    task i2c_write_transaction_n;
        input [6:0] addr;
        input [7:0] data_byte;
        input integer n;
        output addr_ack;
        output integer ack_count;
        integer i;
        reg ack;
        begin
            i2c_start;
            i2c_write_byte({addr, 1'b0}, addr_ack);
            ack_count = 0;
            if (addr_ack) begin
                for (i = 0; i < n; i = i + 1) begin
                    i2c_write_byte(data_byte, ack);
                    if (ack) ack_count = ack_count + 1;
                end
            end
            i2c_stop;
        end
    endtask

    // =======================================================================
    // UART MONITOR
    // =======================================================================

    reg [7:0] uart_queue [0:31];
    integer   q_head, q_tail, q_count;

    task queue_push;
        input [7:0] d;
        begin
            uart_queue[q_tail] = d;
            q_tail  = (q_tail + 1) % 32;
            q_count = q_count + 1;
        end
    endtask

    task queue_pop;
        output [7:0] d;
        begin
            d      = uart_queue[q_head];
            q_head = (q_head + 1) % 32;
            q_count = q_count - 1;
        end
    endtask

    task uart_receive_byte;
        output [7:0] data;
        output       frame_ok;
        integer i;
        begin
            @(negedge uart_tx);        // start bit begins
            #(UART_BIT_TIME/2);        // move to middle of start bit
            if (uart_tx !== 1'b0) begin
                frame_ok = 1'b0;
            end else begin
                data = 8'h00;
                for (i = 0; i < 8; i = i + 1) begin
                    #(UART_BIT_TIME);
                    data[i] = uart_tx;  // LSB first
                end
                #(UART_BIT_TIME);
                frame_ok = (uart_tx === 1'b1); // valid stop bit
            end
        end
    endtask

    initial begin : uart_monitor
        reg [7:0] data;
        reg       frame_ok;
        q_head  = 0;
        q_tail  = 0;
        q_count = 0;
        forever begin
            uart_receive_byte(data, frame_ok);
            if (frame_ok) begin
                queue_push(data);
                $display("[%0t] UART RX captured: 0x%02h", $time, data);
            end else begin
                $display("[%0t] UART RX framing error", $time);
            end
        end
    end

    // =======================================================================
    // CHECK HELPERS
    // =======================================================================

    task check_uart_byte;
        input [7:0] expected;
        input integer timeout_ns;
        integer waited;
        reg [7:0] got;
        begin
            waited = 0;
            while ((q_count == 0) && (waited < timeout_ns)) begin
                #500;
                waited = waited + 500;
            end
            if (q_count == 0) begin
                $display("[%0t] TEST FAIL: timeout waiting for UART byte 0x%02h", $time, expected);
            end else begin
                queue_pop(got);
                if (got == expected)
                    $display("[%0t] TEST PASS: UART byte 0x%02h received as expected", $time, got);
                else
                    $display("[%0t] TEST FAIL: expected 0x%02h, got 0x%02h", $time, expected, got);
            end
        end
    endtask

    task check_no_uart_byte;
        input integer timeout_ns;
        begin
            #timeout_ns;
            if (q_count != 0) begin
                $display("[%0t] TEST FAIL: unexpected UART byte(s) received: %0d in queue", $time, q_count);
                q_head  = 0;
                q_tail  = 0;
                q_count = 0;
            end else begin
                $display("[%0t] TEST PASS: no UART byte received, as expected", $time);
            end
        end
    endtask

    // =======================================================================
    // TEST CASES
    // =======================================================================

    task test_single_byte;
        input [7:0] data;
        reg addr_ack, data_ack;
        begin
            $display("[%0t] --- single byte 0x%02h ---", $time, data);
            i2c_write_transaction_1(OWN_ADDR, data, addr_ack, data_ack);
            if (!addr_ack)
                $display("[%0t] TEST FAIL: address 0x%02h was NOT ACKed", $time, OWN_ADDR);
            else if (!data_ack)
                $display("[%0t] TEST FAIL: data byte 0x%02h was NOT ACKed", $time, data);
            check_uart_byte(data, 200000);
        end
    endtask

    task test_wrong_address;
        input [7:0] data;
        reg addr_ack, data_ack;
        begin
            $display("[%0t] --- wrong address 0x%02h (data 0x%02h) ---", $time, WRONG_ADDR, data);
            i2c_write_transaction_1(WRONG_ADDR, data, addr_ack, data_ack);
            if (addr_ack)
                $display("[%0t] TEST FAIL: wrong address was ACKed (should NACK)", $time);
            else
                $display("[%0t] TEST PASS: wrong address correctly NACKed", $time);
            check_no_uart_byte(100000);
        end
    endtask

    task test_multi_byte;
        reg addr_ack, all_ack;
        begin
            $display("[%0t] --- multi-byte burst (FIFO exercise) ---", $time);
            i2c_write_transaction_5(OWN_ADDR, 8'hA5, 8'h55, 8'h00, 8'hFF, 8'h3C, addr_ack, all_ack);
            if (!addr_ack || !all_ack)
                $display("[%0t] TEST FAIL: burst transaction was not fully ACKed", $time);
            check_uart_byte(8'hA5, 400000);
            check_uart_byte(8'h55, 400000);
            check_uart_byte(8'h00, 400000);
            check_uart_byte(8'hFF, 400000);
            check_uart_byte(8'h3C, 400000);
        end
    endtask

    task test_back_to_back;
        reg addr_ack1, data_ack1, addr_ack2, data_ack2;
        begin
            $display("[%0t] --- back-to-back transactions (start/stop re-arm stress) ---", $time);
            i2c_write_transaction_1(OWN_ADDR, 8'h11, addr_ack1, data_ack1);
            i2c_write_transaction_1(OWN_ADDR, 8'h22, addr_ack2, data_ack2);
            if (!addr_ack1 || !data_ack1 || !addr_ack2 || !data_ack2)
                $display("[%0t] TEST FAIL: back-to-back transactions were not fully ACKed", $time);
            check_uart_byte(8'h11, 200000);
            check_uart_byte(8'h22, 200000);
        end
    endtask

    // Sends 100 identical bytes in one continuous transaction (FIFO
    // depth is 64). 100 is sized with margin above the ~87 bytes the
    // math below says is actually needed - a run of 80 (tried first)
    // was too short and never triggered a NACK, since the UART drains
    // the FIFO concurrently while the burst is still being sent:
    //   write time  = 22,500 ns/byte  (9 I2C clocks)
    //   drain time  = 86,850 ns/byte  (UART frame + bridge_controller)
    //   net fill    = 1/22500 - 1/86850 ~= 32,930 bytes/s
    //   time to net-accumulate 64 bytes ~= 64 / 32930 ~= 1.94 ms
    //   bytes SENT in that time ~= 1.94ms / 22,500ns ~= 87 bytes
    // After the burst, this also waits for every ACKed byte to actually
    // arrive on uart_tx (not just checks the I2C-side ACK/NACK), which
    // is a stronger check than only counting NACKs.
    task test_fifo_overflow;
        reg addr_ack;
        integer ack_count;
        integer waited;
        integer drain_timeout;
        begin
            $display("[%0t] --- FIFO overflow: 100 bytes into a 64-deep FIFO ---", $time);
            i2c_write_transaction_n(OWN_ADDR, 8'hEE, 100, addr_ack, ack_count);
            $display("[%0t] %0d of 100 bytes were ACKed", $time, ack_count);

            if (!addr_ack)
                $display("[%0t] TEST FAIL: address was NOT ACKed", $time);
            else if (ack_count >= 100)
                $display("[%0t] TEST FAIL: all 100 bytes ACKed - FIFO-full should have caused at least one NACK", $time);
            else if (ack_count < 64)
                $display("[%0t] TEST FAIL: fewer than 64 (FIFO depth) bytes ACKed - unexpected", $time);
            else
                $display("[%0t] FIFO-full correctly NACKed once capacity was reached", $time);

            // now confirm every ACKed byte actually makes it out over UART
            drain_timeout = ack_count * 100000 + 300000;
            waited = 0;
            while ((q_count < ack_count) && (waited < drain_timeout)) begin
                #2000;
                waited = waited + 2000;
            end
            if (q_count == ack_count) begin
                $display("[%0t] TEST PASS: all %0d ACKed bytes drained correctly over UART", $time, ack_count);
                q_head = 0; q_tail = 0; q_count = 0; // clear for the next test
            end else begin
                $display("[%0t] TEST FAIL: expected %0d bytes on UART, only %0d arrived before timeout", $time, ack_count, q_count);
            end
        end
    endtask

    // =======================================================================
    // MAIN SEQUENCE - runs by itself, no user input needed
    // =======================================================================

    initial begin
        wait (rst_in_n == 1'b1);
        #(CLK_PERIOD*20);

        $display("=== TEST 1: single byte 0xA5 ===");
        test_single_byte(8'hA5);

        $display("=== TEST 2: single byte 0x3C ===");
        test_single_byte(8'h3C);

        $display("=== TEST 3: single byte 0x00 ===");
        test_single_byte(8'h00);

        $display("=== TEST 4: single byte 0xFF ===");
        test_single_byte(8'hFF);

        $display("=== TEST 5: single byte 0xC3 ===");
        test_single_byte(8'hC3);

        $display("=== TEST 6: wrong address 0x51 ===");
        test_wrong_address(8'h12);

        $display("=== TEST 7: multi-byte burst ===");
        test_multi_byte;

        $display("=== TEST 8: back-to-back transactions ===");
        test_back_to_back;

        $display("=== TEST 9: FIFO overflow (NACK-on-full) ===");
        test_fifo_overflow;

        #50000;
        $display("=== ALL TESTS COMPLETE ===");
        $stop;
    end

endmodule
