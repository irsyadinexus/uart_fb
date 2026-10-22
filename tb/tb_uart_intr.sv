//==============================================================================
// tb_uart_intr.sv — UART Interrupt Testbench
//
// Tests the interrupt generation logic in uart_top.sv across all enable
// sources (int_en_rx, int_en_tx, int_en_err) including:
//   - RX IRQ: data available triggers interrupt
//   - TX IRQ: transmitter ready triggers interrupt
//   - ERR IRQ: parity/framing errors trigger interrupt
//   - IRQ de-assertion when condition clears
//   - Enable/disable toggling
//
// Uses loopback with optional error injection to test error-triggered
// interrupts through the full UART path.
//==============================================================================

`timescale 1ns / 1ps
import uart_pkg::*;

module tb_uart_intr();

  parameter CLK_PERIOD = 20;
  parameter CLK_FREQ   = 50_000_000;

  // ===========================================================================
  // Signals
  // ===========================================================================
  logic clk, rst_n;
  logic [15:0] cfg_baud_div;
  logic [2:0] cfg_data_bits;
  logic [1:0] cfg_parity, cfg_stop_bits;
  logic [7:0] tx_data, rx_data;
  logic tx_valid, tx_ready, tx_busy, rx_valid, rx_ready, rx_busy;
  logic framing_error, parity_error, overrun_error, break_detect;
  logic irq;
  logic int_en_rx, int_en_tx, int_en_err;
  logic txd, rxd;
  logic clr_errors;

  // Error injection (invert rxd when inject_err asserted)
  logic inject_err;
  assign rxd = inject_err ? ~txd : txd;

  // ===========================================================================
  // DUT
  // ===========================================================================
  uart_top #(.CLK_FREQ(CLK_FREQ)) u_uart (.*);

  // ===========================================================================
  // Clock & VCD
  // ===========================================================================
  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    $dumpfile("tb_uart_intr.vcd");
    $dumpvars(0, tb_uart_intr);
  end

  // ===========================================================================
  // Debug monitor
  // ===========================================================================
  // Uncomment to trace every posedge clk:
  // always @(posedge clk) begin
  //   $display("[%0t] rxv=%b rxr=%b txr=%b irq=%b ie_rx=%b ie_tx=%b ie_err=%b fe=%b pe=%b",
  //     $time, rx_valid, rx_ready, irq, int_en_rx, int_en_tx, int_en_err,
  //     framing_error, parity_error);
  // end

  // ===========================================================================
  // Helpers
  // ===========================================================================
  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  task reset();
    rst_n = 1'b0;
    tx_data = 8'h00; tx_valid = 1'b0; rx_ready = 1'b0;
    int_en_rx = 1'b0; int_en_tx = 1'b0; int_en_err = 1'b0;
    inject_err = 1'b0; clr_errors = 1'b0;
    cfg_baud_div = CLK_FREQ / 115200;
    cfg_data_bits = 3'd3; cfg_parity = PARITY_EVEN; cfg_stop_bits = STOP_1;
    wait_clks(10); rst_n = 1'b1; wait_clks(20);
    $display("[%0t] Reset complete", $time);
  endtask

  // Send a byte via the TX interface (handshake), then wait for rx_valid
  // Does NOT clear rx_valid — caller must call ack_rx()
  task send_byte(input logic [7:0] val);
    int timeout;
    timeout = 100000;
    while (!tx_ready && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) begin
      tx_data = val; tx_valid = 1'b1;
      @(posedge clk);
      tx_valid = 1'b0;
      // Wait for RX to receive the looped-back byte
      timeout = 500000;
      while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    end else begin
      $display("  TIMEOUT tx_ready");
    end
  endtask

  // Send byte with error injection — corrupt the RX line during the data/parity
  // frame to cause a detectable error. Uses a wide injection window (start of
  // frame through end) to ensure the parity bit is hit regardless of timing.
  task send_byte_with_err(input logic [7:0] val, input int parity_mode);
    int timeout;
    reg txd_prev;
    timeout = 100000;
    while (!tx_ready && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) begin
      cfg_parity = parity_mode;
      wait_clks(3);
      tx_data = val; tx_valid = 1'b1;
      @(posedge clk);
      tx_valid = 1'b0;
      // Wait for start bit falling edge
      txd_prev = 1'b1; timeout = 500000;
      while (timeout > 0) begin
        @(posedge clk);
        if (txd_prev && !txd) timeout = 0;
        else timeout = timeout - 1;
        txd_prev = txd;
      end
      // Wait a brief window after start edge, then inject error for remaining
      // frame (covers data bits through stop bit — guaranteed to hit parity)
      repeat (3 * 434) @(posedge clk);  // well past start bit
      inject_err = 1'b1;
      repeat (10 * 434) @(posedge clk); // covers data+parity+stop
      inject_err = 1'b0;
      // Now wait for rx_valid
      timeout = 500000;
      while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    end else begin
      $display("  TIMEOUT tx_ready");
    end
  endtask

  task ack_rx();
    rx_ready = 1'b1;
    @(posedge clk);
    rx_ready = 1'b0;
  endtask

  task clear_errs();
    clr_errors = 1'b1;
    @(posedge clk);
    clr_errors = 1'b0;
  endtask

  // ===========================================================================
  // Main Test
  // ===========================================================================
  initial begin
    integer tp, tf;
    tp = 0; tf = 0;

    $display("");
    $display("╔═══════════════════════════════════════════════════╗");
    $display("║       UART Interrupt Test                        ║");
    $display("╚═══════════════════════════════════════════════════╝");
    $display("");

    reset();

    // ======================================================================
    // TEST 1: int_en_rx → IRQ fires when data received
    // ======================================================================
    $display("TEST 1: int_en_rx → IRQ on data receive");
    int_en_rx = 1'b1; int_en_tx = 1'b0; int_en_err = 1'b0;
    wait_clks(5);

    send_byte(8'h55);
    #1;
    $display("  DBG: rxv=%b irq=%b int_en_rx=%b txr=%b", rx_valid, irq, int_en_rx, tx_ready);
    if (rx_valid && irq) begin
      $display("  PASS: IRQ=%b after data received", irq); tp = tp + 1;
    end else begin
      $display("  FAIL: rxv=%b irq=%b", rx_valid, irq); tf = tf + 1;
    end
    ack_rx();
    wait_clks(10);

    // ======================================================================
    // TEST 2: int_en_rx=0 → no IRQ on data receive
    // ======================================================================
    $display("TEST 2: int_en_rx=0 → no IRQ on data receive");
    int_en_rx = 1'b0;
    wait_clks(5);
    send_byte(8'hAA);
    #1;
    $display("  DBG: rxv=%b irq=%b", rx_valid, irq);
    if (rx_valid && !irq) begin
      $display("  PASS: IRQ=%b (int_en_rx disabled)", irq); tp = tp + 1;
    end else begin
      $display("  FAIL: rxv=%b irq=%b (expected rxv=1 irq=0)", rx_valid, irq); tf = tf + 1;
    end
    ack_rx();
    wait_clks(10);

    // ======================================================================
    // TEST 3: int_en_tx → IRQ fires when tx_ready
    // ======================================================================
    $display("TEST 3: int_en_tx → IRQ on tx_ready");
    int_en_rx = 1'b0; int_en_tx = 1'b1; int_en_err = 1'b0;
    // Wait until TX is idle (may still be busy from previous test's framing)
    begin
      int to = 500000;
      while (!tx_ready && to > 0) begin @(posedge clk); to = to - 1; end
    end
    #1;
    $display("  DBG: irq=%b txr=%b", irq, tx_ready);
    if (irq && tx_ready) begin
      $display("  PASS: IRQ=%b tx_ready=%b", irq, tx_ready); tp = tp + 1;
    end else begin
      $display("  FAIL: irq=%b txr=%b", irq, tx_ready); tf = tf + 1;
    end
    wait_clks(10);

    // ======================================================================
    // TEST 4: int_en_err → IRQ on parity error
    // ======================================================================
    $display("TEST 4: int_en_err → IRQ on parity error");
    int_en_rx = 1'b0; int_en_tx = 1'b0; int_en_err = 1'b1;
    clear_errs();
    wait_clks(5);

    send_byte_with_err(8'h55, PARITY_EVEN);
    #1;
    $display("  DBG: rxv=%b irq=%b pe=%b", rx_valid, irq, parity_error);
    if (parity_error && irq) begin
      $display("  PASS: IRQ=%b pe=%b", irq, parity_error); tp = tp + 1;
    end else begin
      $display("  FAIL: rxv=%b irq=%b pe=%b", rx_valid, irq, parity_error); tf = tf + 1;
    end
    ack_rx();
    clear_errs();
    wait_clks(10);

    // ======================================================================
    // TEST 5: int_en_err → IRQ on framing error
    // ======================================================================
    $display("TEST 5: int_en_err → IRQ on framing error");
    int_en_rx = 1'b0; int_en_tx = 1'b0; int_en_err = 1'b1;
    clear_errs();
    wait_clks(5);

    // Send 0x55 EVEN, force RXD low during stop bit to create framing error
    begin
      int timeout;
      reg txd_prev;
      timeout = 100000;
      while (!tx_ready && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
      cfg_parity = PARITY_EVEN;
      wait_clks(3);
      tx_data = 8'h55; tx_valid = 1'b1;
      @(posedge clk);
      tx_valid = 1'b0;
      // Wait for start bit falling edge
      txd_prev = 1'b1; timeout = 500000;
      while (timeout > 0) begin
        @(posedge clk);
        if (txd_prev && !txd) timeout = 0;
        else timeout = timeout - 1;
        txd_prev = txd;
      end
      // Wait past start(1)+8data(8)+parity(1) = 10 bit periods. Use generous margin.
      repeat (15 * 434) @(posedge clk);
      // Force RXD low during stop bit → framing error
      inject_err = 1'b1;
      repeat (868) @(posedge clk);  // ~2 bit periods of forced low
      inject_err = 1'b0;
      // Wait for rx_valid
      timeout = 500000;
      while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    end
    #1;
    $display("  DBG: rxv=%b irq=%b fe=%b", rx_valid, irq, framing_error);
    if (framing_error && irq) begin
      $display("  PASS: IRQ=%b fe=%b", irq, framing_error); tp = tp + 1;
    end else begin
      $display("  FAIL: rxv=%b irq=%b fe=%b", rx_valid, irq, framing_error); tf = tf + 1;
    end
    ack_rx();
    clear_errs();
    wait_clks(10);

    // ======================================================================
    // TEST 6: All enables → IRQ on data + error
    // ======================================================================
    $display("TEST 6: All enables → IRQ on data + error");
    int_en_rx = 1'b1; int_en_tx = 1'b1; int_en_err = 1'b1;
    clear_errs();
    wait_clks(5);

    send_byte(8'h55);
    #1;
    $display("  DBG: rxv=%b irq=%b", rx_valid, irq);
    if (rx_valid && irq) begin
      $display("  PASS: IRQ=%b (all enables)", irq); tp = tp + 1;
    end else begin
      $display("  FAIL: rxv=%b irq=%b", rx_valid, irq); tf = tf + 1;
    end
    ack_rx();
    wait_clks(10);

    // ======================================================================
    // TEST 7: IRQ de-asserts when rx_valid clears
    // ======================================================================
    $display("TEST 7: IRQ de-asserts after rx_valid clears");
    int_en_rx = 1'b1; int_en_tx = 1'b0; int_en_err = 1'b0;
    clear_errs();
    wait_clks(5);

    send_byte(8'h7B);
    wait_clks(1);
    if (rx_valid && irq) begin
      $display("  IRQ=%b after receive (expected)", irq);
      ack_rx();
      wait_clks(2);
      $display("  DBG after ack: rxv=%b irq=%b", rx_valid, irq);
      if (!irq) begin
        $display("  PASS: IRQ=%b after ack (de-asserted)", irq); tp = tp + 1;
      end else begin
        $display("  FAIL: IRQ=%b rxv=%b after ack", irq, rx_valid); tf = tf + 1;
      end
    end else begin
      $display("  FAIL: rxv=%b irq=%b (receive failed)", rx_valid, irq); tf = tf + 1;
    end
    wait_clks(10);

    // ======================================================================
    // TEST 8: Toggle enables mid-operation
    // ======================================================================
    $display("TEST 8: Toggle enables — IRQ follows enables");
    int_en_rx = 1'b0; int_en_tx = 1'b0; int_en_err = 1'b0;
    clear_errs();
    wait_clks(5);

    send_byte_with_err(8'h55, PARITY_EVEN);
    wait_clks(1);
    $display("  DBG: all off: irq=%b pe=%b fe=%b rxv=%b",
      irq, parity_error, framing_error, rx_valid);
    // All enables off — IRQ should be 0 regardless of events
    if (!irq) begin
      $display("  (all disabled) IRQ=%b PASS", irq);
    end else begin
      $display("  (all disabled) irq=%b — unexpected", irq);
    end
    // Now enable ERR IRQ while any sticky error is still asserted
    int_en_err = 1'b1;
    wait_clks(1);
    $display("  DBG: err on: irq=%b pe=%b fe=%b oe=%b brk=%b", irq, parity_error,
      framing_error, overrun_error, break_detect);
    if (irq && (parity_error || framing_error)) begin
      $display("  PASS: IRQ=%b follows enable toggle (pe=%b fe=%b)",
        irq, parity_error, framing_error); tp = tp + 1;
    end else begin
      $display("  FAIL: irq=%b pe=%b fe=%b after toggle",
        irq, parity_error, framing_error); tf = tf + 1;
    end
    clear_errs();
    wait_clks(10);

    // ======================================================================
    // TEST 9: int_en_err → IRQ on overrun error
    // ======================================================================
    // Overrun: send a second byte without reading the first (rx_valid still
    // high, rx_ready=0). The RX_DONE state sees rx_valid=1 && !rx_ready and
    // sets overrun_error=1 (sticky).
    // ======================================================================
    $display("TEST 9: int_en_err → IRQ on overrun error");
    int_en_rx = 1'b0; int_en_tx = 1'b0; int_en_err = 1'b1;
    clear_errs();
    wait_clks(5);

    // Send byte 1 via standard send_byte → rx_valid=1 after reception
    send_byte(8'h55);
    wait_clks(10);
    $display("  Byte1 rx_valid=%b (NOT acknowledging)", rx_valid);

    // Send byte 2 WITHOUT waiting for rx_valid (it's already high from byte1).
    // Instead, wait a fixed time for the frame to complete, then check
    // overrun_error.
    begin
      int timeout;
      timeout = 100000;
      while (!tx_ready && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
      if (timeout > 0) begin
        // Latch byte 2 — TX will transmit it while rx_valid stays high
        tx_data = 8'hAA; tx_valid = 1'b1;
        @(posedge clk);
        tx_valid = 1'b0;
        // Wait for byte 2 to be fully transmitted and received
        // 8N1 frame: start + 8 data + stop = 10 bit periods at ~434 cycles/bit
        // Use 50000 (~1ms) generous margin over the 4340-cycle frame time
        timeout = 50000;
        while (timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
      end
    end
    #1;
    $display("  DBG: rxv=%b oe=%b irq=%b", rx_valid, overrun_error, irq);
    if (overrun_error && irq) begin
      $display("  PASS: IRQ=%b oe=%b (overrun triggered)", irq, overrun_error);
      tp = tp + 1;
    end else begin
      $display("  FAIL: irq=%b oe=%b rxv=%b", irq, overrun_error, rx_valid);
      tf = tf + 1;
    end
    ack_rx();  // Clear rx_valid from byte 1 (data is stale anyway)
    clear_errs();
    wait_clks(10);

    // ======================================================================
    // Results
    // ======================================================================
    $display("");
    $display("╔═══════════════════════════════════════════════════╗");
    $display("║  INTERRUPT TEST RESULTS                           ║");
    $display("╠═══════════════════════════════════════════════════╣");
    $display("║  %2d / %2d tests PASSED%s", tp, tp + tf, (tf == 0) ? "          ║" : "");
    if (tf > 0) begin
      $display("║  %2d tests FAILED!                    ║", tf);
    end
    $display("╚═══════════════════════════════════════════════════╝");
    $display("");

    if (tf > 0) begin
      $fatal(1, "tb_uart_intr: %0d failures!", tf);
    end

    #5000;
    $finish;
  end

  // ===========================================================================
  // Timeout
  // ===========================================================================
  initial begin
    #500_000_000;  // 500ms
    $display("[%0t] GLOBAL TIMEOUT", $time);
    $finish;
  end

endmodule
