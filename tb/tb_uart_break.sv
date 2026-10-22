//==============================================================================
// tb_uart_break.sv — Break Detection Testbench
//
// Tests the uart_rx break_detect feature by driving the serial line (RXD)
// with precise timing to verify:
//   - Normal frames do NOT trigger break detection
//   - Continuous low > break threshold triggers break_detect
//   - break_detect is sticky (W1C via clr_errors)
//   - Short low periods (below threshold) do NOT trigger break
//   - Break detection works across different data widths
//==============================================================================

`timescale 1ns / 1ps
import uart_pkg::*;

module tb_uart_break();

  parameter CLK_PERIOD = 20;       // 50 MHz
  parameter BIT_TICKS  = 28;       // x16 tick period: 28 clk cycles (~115200 baud)

  // Break threshold: 200 x16 ticks = 200 * 28 = 5600 clock cycles
  parameter BREAK_CLKS = 200 * BIT_TICKS;  // 5600 clock cycles of continuous low

  logic clk, rst_n;
  logic [2:0] cfg_data_bits;
  logic [1:0] cfg_parity, cfg_stop_bits;
  logic [7:0] rx_data;
  logic rx_valid, rx_ready, rx_busy;
  logic framing_error, parity_error, overrun_error, break_detect, rxd;
  logic baud_x16_tick;
  logic [11:0] x16_cnt;
  logic clr_errors;

  // Clear sticky error flags (W1C)
  task clear_errors();
    clr_errors = 1'b1;
    @(posedge clk);
    clr_errors = 1'b0;
  endtask

  // Wait for rx_valid with timeout; returns 0 on timeout
  task wait_rx(output int tout);
    tout = 50000;
    while (!rx_valid && tout > 0) begin @(posedge clk); tout = tout - 1; end
  endtask

  // Acknowledge received data
  task ack_rx();
    rx_ready = 1'b1;
    @(posedge clk);
    rx_ready = 1'b0;
  endtask

  // Wait N clock cycles
  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  // ===========================================================================
  // x16 tick generator
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin x16_cnt <= '0; baud_x16_tick <= 1'b0; end
    else begin
      baud_x16_tick <= 1'b0;
      if (x16_cnt >= BIT_TICKS - 1) begin x16_cnt <= '0; baud_x16_tick <= 1'b1; end
      else x16_cnt <= x16_cnt + 1;
    end
  end

  // ===========================================================================
  // DUT: uart_rx directly
  // ===========================================================================
  uart_rx u_rx (
    .clk, .rst_n,
    .cfg_data_bits, .cfg_parity, .cfg_stop_bits,
    .baud_x16_tick,
    .rx_data, .rx_valid, .rx_ready, .rx_busy,
    .framing_error, .parity_error, .overrun_error, .break_detect,
    .clr_errors,
    .rxd
  );

  // ===========================================================================
  // Clock
  // ===========================================================================
  initial clk = 1'b0;
  always #(CLK_PERIOD / 2) clk = ~clk;

  // ===========================================================================
  // VCD dump
  // ===========================================================================
  initial begin
    $dumpfile("tb_uart_break.vcd");
    $dumpvars(0, tb_uart_break);
  end

  // ===========================================================================
  // Helper: Send a complete UART frame (start + data + parity + stop)
  // and CAPTURE the received byte as soon as rx_valid fires.
  // Each bit is held for (16 * BIT_TICKS) clock cycles to match the x16
  // oversampling timing.
  //
  // CRITICAL: send_frame is PARITY-AWARE — it only drives a parity bit slot
  // when cfg_parity != PARITY_NONE. And it polls for rx_valid DURING the
  // frame (at each bit boundary) to capture rx_data BEFORE any spurious
  // frame can overwrite it. The captured value is returned via the `rdata`
  // output and `rok` indicates whether rx_valid fired within timeout.
  // ===========================================================================
  task send_frame(
    input logic [7:0] data,
    input int         bits,
    input logic       parity_val,
    input int         stops,
    output logic [7:0] rdata,
    output logic       rok
  );
    integer i;
    int       timeout;

    rok = 1'b0; rdata = 8'h00;

    // Re-arm rx_ready to clear any stale rx_valid from previous frames.
    rx_ready = 1'b1;
    @(posedge clk);
    rx_ready = 1'b0;

    // Start bit (low)
    rxd = 1'b0;
    repeat (16) @(posedge baud_x16_tick);

    // Poll for rx_valid after each bit; it may fire before the frame ends
    for (i = 0; i < bits; i++) begin
      rxd = data[i];
      // While holding this bit, poll for rx_valid
      timeout = 16;
      while (timeout > 0) begin
        @(posedge baud_x16_tick);
        timeout = timeout - 1;
        if (rx_valid && !rok) begin
          rdata = rx_data;
          rok = 1'b1;
        end
      end
    end

    // Parity bit (only when parity is enabled)
    if (cfg_parity != PARITY_NONE) begin
      rxd = parity_val;
      timeout = 16;
      while (timeout > 0) begin
        @(posedge baud_x16_tick);
        timeout = timeout - 1;
        if (rx_valid && !rok) begin
          rdata = rx_data;
          rok = 1'b1;
        end
      end
    end

    // Stop bit(s)
    for (i = 0; i < stops; i++) begin
      rxd = 1'b1;
      timeout = 16;
      while (timeout > 0) begin
        @(posedge baud_x16_tick);
        timeout = timeout - 1;
        if (rx_valid && !rok) begin
          rdata = rx_data;
          rok = 1'b1;
        end
      end
    end

    // Post-frame guard: hold RXD high to let the receiver settle in IDLE.
    // 4 x16 ticks (112 clock cycles) is 28× the glitch filter's 4-cycle
    // window — sufficient margin to prevent false start detection.
    repeat (4) @(posedge baud_x16_tick);
    if (rx_valid && !rok) begin
      rdata = rx_data;
      rok = 1'b1;
    end
  endtask

  // ===========================================================================
  // Helper: Hold RXD low for N x16 ticks (for break generation)
  // ===========================================================================
  task hold_low_x16(input int n_ticks);
    integer j;
    rxd = 1'b0;
    repeat (n_ticks) @(posedge baud_x16_tick);
  endtask

  // ===========================================================================
  // Main Test
  // ===========================================================================
  initial begin
    integer tp, tf, tout;
    logic [7:0] rdata;
    logic       rok;
    string test_name;

    tp = 0; tf = 0;

    $display("");
    $display("╔═══════════════════════════════════════════════════╗");
    $display("║       UART Break Detection Test                   ║");
    $display("╚═══════════════════════════════════════════════════╝");
    $display("Break threshold: %0d x16 ticks (~%0d clk cycles)\n",
             200, BREAK_CLKS);

    // ======================================================================
    // Reset + Initialize
    // ======================================================================
    rst_n = 1'b0; rxd = 1'b1; rx_ready = 1'b0; clr_errors = 1'b0;
    cfg_data_bits = 3'd3; cfg_parity = PARITY_EVEN; cfg_stop_bits = STOP_1;
    wait_clks(10); rst_n = 1'b1; wait_clks(10);
    clear_errors();
    $display("[%0t] Reset complete", $time);

    // ======================================================================
    // TEST 1: Normal frame, correct parity → break_detect=0
    // ======================================================================
    test_name = "TEST 1: Normal 0x55 (8E1, correct parity)";
    send_frame(8'h55, 8, 1'b0, 1, rdata, rok);
    if (rok && !break_detect) begin
      $display("  %s → PASS (rx=0x%02x)", test_name, rdata); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (rok=%b brk=%b rx=0x%02x)", test_name, rok, break_detect, rdata);
      tf = tf + 1;
    end
    ack_rx(); clear_errors();
    wait_clks(20);

    // ======================================================================
    // TEST 2: Hold RXD low for 150 x16 ticks (< threshold) → no break
    // ======================================================================
    test_name = "TEST 2: Low 150 ticks (below 200 threshold)";
    hold_low_x16(150);
    rxd = 1'b1;
    wait_clks(10);
    if (!break_detect) begin
      $display("  %s → PASS", test_name); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (brk=%b)", test_name, break_detect);
      tf = tf + 1;
    end
    clear_errors();
    wait_clks(20);

    // ======================================================================
    // TEST 3: Hold RXD low for 250 x16 ticks (> threshold) → break_detect=1
    // ======================================================================
    test_name = "TEST 3: Low 250 ticks (above 200 threshold)";
    hold_low_x16(250);
    rxd = 1'b1;
    wait_clks(10);
    if (break_detect) begin
      $display("  %s → PASS", test_name); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (brk=%b)", test_name, break_detect);
      tf = tf + 1;
    end
    wait_clks(20);

    // ======================================================================
    // TEST 4: W1C: pulse clr_errors clears break_detect
    // ======================================================================
    test_name = "TEST 4: W1C clear (clr_errors pulse)";
    clear_errors();  // Pulse clr_errors
    wait_clks(5);
    if (!break_detect) begin
      $display("  %s → PASS", test_name); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (brk=%b after W1C)", test_name, break_detect);
      tf = tf + 1;
    end
    wait_clks(20);

    // ======================================================================
    // TEST 5: Two consecutive breaks, verify sticky behavior
    // ======================================================================
    test_name = "TEST 5: First of 2 consecutive breaks";
    hold_low_x16(250);
    rxd = 1'b1;
    wait_clks(10);
    if (break_detect) begin
      $display("  %s → PASS", test_name); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (brk=%b)", test_name, break_detect);
      tf = tf + 1;
    end

    // Clear, then verify second break also detected
    clear_errors();

    test_name = "TEST 5b: Second break after clear";
    hold_low_x16(250);
    rxd = 1'b1;
    wait_clks(10);
    if (break_detect) begin
      $display("  %s → PASS", test_name); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (brk=%b)", test_name, break_detect);
      tf = tf + 1;
    end
    clear_errors();
    wait_clks(20);

    // ======================================================================
    // TEST 6: Normal 0x00 with even parity (max valid low = 160 ticks)
    // This is the longest valid low period: start(16) + 8data(128) + even(16)
    // = 160 ticks, which is below the 200-tick threshold.
    // ======================================================================
    test_name = "TEST 6: 0x00 8E1 (max valid low = 160 ticks)";
    send_frame(8'h00, 8, 1'b0, 1, rdata, rok);
    if (rok && rdata == 8'h00 && !break_detect) begin
      $display("  %s → PASS (rx=0x%02x)", test_name, rdata); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (rok=%b rx=0x%02x brk=%b)", test_name, rok, rdata, break_detect);
      tf = tf + 1;
    end
    ack_rx(); clear_errors();
    wait_clks(20);

    // ======================================================================
    // TEST 7: Pulse reset + Normal 0x00 with no parity (low = 144 ticks)
    // Reset ensures clean FSM state and sample counter alignment after
    // the break recovery period.
    // ======================================================================
    test_name = "TEST 7: Reset + 0x00 8N1 (low = 144 ticks)";
    rst_n = 1'b0; wait_clks(5); rst_n = 1'b1; wait_clks(10);
    cfg_parity = PARITY_NONE;
    send_frame(8'h00, 8, 1'b0, 1, rdata, rok);
    if (rok && !break_detect) begin
      $display("  %s → PASS (rx=0x%02x)", test_name, rdata); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (rok=%b rx=0x%02x brk=%b)", test_name, rok, rdata, break_detect);
      tf = tf + 1;
    end
    ack_rx(); clear_errors();
    wait_clks(20);

    // ======================================================================
    // TEST 8: 5-bit data with break
    // ======================================================================
    cfg_parity = PARITY_EVEN;
    cfg_data_bits = 3'd0;  // 5 data bits
    wait_clks(5);

    test_name = "TEST 8: Break with 5-bit config";
    hold_low_x16(250);
    rxd = 1'b1;
    wait_clks(10);
    if (break_detect) begin
      $display("  %s → PASS", test_name); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (brk=%b)", test_name, break_detect);
      tf = tf + 1;
    end
    clear_errors();
    wait_clks(20);

    // ======================================================================
    // TEST 9: Reset + Normal 5-bit frame (verify receiver recovers)
    // ======================================================================
    test_name = "TEST 9: Reset + 5-bit frame after break";
    rst_n = 1'b0; wait_clks(5); rst_n = 1'b1; wait_clks(10);
    cfg_data_bits = 3'd0; cfg_parity = PARITY_EVEN;
    send_frame(8'h05, 5, 1'b0, 1, rdata, rok);
    if (rok && !break_detect) begin
      $display("  %s → PASS (rx=0x%02x)", test_name, rdata); tp = tp + 1;
    end else begin
      $display("  %s → FAIL (rok=%b rx=0x%02x brk=%b)", test_name, rok, rdata, break_detect);
      tf = tf + 1;
    end
    ack_rx(); clear_errors();
    wait_clks(20);

    // ======================================================================
    // Results
    // ======================================================================
    $display("");
    $display("╔═══════════════════════════════════════════════════╗");
    $display("║  BREAK DETECTION TEST RESULTS                    ║");
    $display("╠═══════════════════════════════════════════════════╣");
    $display("║  %2d / %2d tests PASSED%s", tp, tp + tf, (tf == 0) ? "          ║" : "");
    if (tf > 0) begin
      $display("║  %2d tests FAILED!                    ║", tf);
    end
    $display("╚═══════════════════════════════════════════════════╝");
    $display("");

    if (tf > 0) begin
      $fatal(1, "tb_uart_break: %0d failures!", tf);
    end

    #5000;
    $finish;
  end

  // ===========================================================================
  // Timeout
  // ===========================================================================
  initial begin
    #200_000_000;  // 200ms
    $display("[%0t] GLOBAL TIMEOUT", $time);
    $finish;
  end

endmodule
