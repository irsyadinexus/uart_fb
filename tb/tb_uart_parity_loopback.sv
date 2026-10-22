//==============================================================================
// tb_uart_parity_loopback.sv — Loopback Parity Error Injection Test
// Uses uart_top with rxd=txd loopback. Injects a bit error on the loopback
// line during the parity bit period to verify parity_error detection.
//==============================================================================

`timescale 1ns / 1ps
import uart_pkg::*;

module tb_uart_parity_loopback();

  parameter CLK_FREQ   = 50_000_000;
  parameter BIT_PERIOD = 434;  // 50MHz / 115200

  logic clk, rst_n;
  logic [15:0] cfg_baud_div;
  logic [2:0] cfg_data_bits;
  logic [1:0] cfg_parity, cfg_stop_bits;
  logic [7:0] tx_data, rx_data;
  logic tx_valid, tx_ready, tx_busy, rx_valid, rx_ready, rx_busy;
  logic framing_error, parity_error, overrun_error, break_detect, txd, rxd;
  logic irq;
  logic int_en_rx, int_en_tx, int_en_err;
  logic clr_errors;  // W1C: pulse high to clear sticky error flags

  // Loopback with error injection (invert rxd when inject_err asserted)
  logic inject_err;
  assign rxd = inject_err ? ~txd : txd;

  uart_top #(.CLK_FREQ(CLK_FREQ)) u_uart (.*);
  assign clr_errors = 1'b0;

  initial clk = 0;
  always #10 clk = ~clk;

  initial begin
    $dumpfile("tb_uart_parity_loopback.vcd");
    $dumpvars(0, tb_uart_parity_loopback);
  end

  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  // Send a byte via uart_top. Optionally inject a parity error.
  // db: cfg_data_bits value (0=5, 1=6, 2=7, 3=8 bits)
  // inject: 1 to flip the parity bit on the loopback
  //
  // Timing: Detects the actual start bit (txd falling edge) and counts
  // from there, eliminating TX start delay uncertainty. The parity bit
  // starts exactly (1 + num_bits) * BIT_PERIOD cycles after the falling edge.
  // Injecting for 1 BIT_PERIOD covers ONLY the parity bit — no data or stop
  // bit corruption.
  task send_byte(input logic [7:0] val, input int db, input int pm, input int inject);
    int timeout;
    int num_bits;     // Actual data bit count

    num_bits = db + 5;  // 0→5, 1→6, 2→7, 3→8

    // Configure and reset to clear any stale state from previous tests
    rst_n = 1'b0;
    cfg_data_bits = db;
    cfg_parity    = pm;
    cfg_stop_bits = STOP_1;
    inject_err = 1'b0;
    wait_clks(5);
    rst_n = 1'b1;
    wait_clks(10);

    // Wait for tx_ready
    timeout = 50000;
    while (!tx_ready && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end

    // Send byte via TX interface
    tx_data = val;
    tx_valid = 1'b1;
    @(posedge clk);
    tx_valid = 1'b0;

    // Poll for start bit falling edge (in-line, avoids delta-cycle ambiguity)
    // Once detected, we have a precise reference point to count from.
    begin
      reg txd_prev;
      txd_prev = 1'b1;
      timeout = 50000;
      while (timeout > 0) begin
        @(posedge clk);
        if (txd_prev && !txd) timeout = 0;  // falling edge detected, break
        else timeout = timeout - 1;
        txd_prev = txd;
      end
    end

    // Wait for parity bit: start(1) + data_bits bit periods from falling edge.
    // The edge is detected at posedge N, but repeat counts from posedge N+1,
    // so add 1 to compensate for the 1-cycle offset.
    repeat ((1 + num_bits) * BIT_PERIOD + 1) @(posedge clk);

    // Inject error for exactly 1 bit period (covers the parity bit only)
    if (inject && pm != PARITY_NONE) begin
      inject_err = 1'b1;
      repeat (BIT_PERIOD) @(posedge clk);
      inject_err = 1'b0;
    end
  endtask

  initial begin
    integer tp, tf, timeout;
    reg waited;

    tp = 0; tf = 0;
    $display("=== UART Loopback Parity Error Injection Test ===\n");

    // Initialize
    rst_n = 1'b0; tx_valid = 1'b0; rx_ready = 1'b1; inject_err = 1'b0;
    int_en_rx = 1'b0; int_en_tx = 1'b0; int_en_err = 1'b0;
    cfg_baud_div = CLK_FREQ / 115200;
    cfg_data_bits = 3'd3; cfg_parity = PARITY_EVEN; cfg_stop_bits = STOP_1;
    wait_clks(10); rst_n = 1'b1; wait_clks(10);

    // ===== CORRECT PARITY (no injection) =====
    // These verify that without injection, parity_error stays 0

    // TEST 1: 0x55 EVEN, no injection -> pe=0
    send_byte(8'h55, 3'd3, PARITY_EVEN, 0);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && !parity_error) begin $display("  TEST 1 PASS (0x55 EVEN no inject)"); tp=tp+1; end
    else begin $display("  TEST 1 FAIL pe=%b", parity_error); tf=tf+1; end

    // TEST 2: 0xAA ODD, no injection -> pe=0
    send_byte(8'hAA, 3'd3, PARITY_ODD, 0);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && !parity_error) begin $display("  TEST 2 PASS (0xAA ODD no inject)"); tp=tp+1; end
    else begin $display("  TEST 2 FAIL pe=%b", parity_error); tf=tf+1; end

    // TEST 3: 0x01 EVEN, no injection -> pe=0
    send_byte(8'h01, 3'd3, PARITY_EVEN, 0);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && !parity_error) begin $display("  TEST 3 PASS (0x01 EVEN no inject)"); tp=tp+1; end
    else begin $display("  TEST 3 FAIL pe=%b", parity_error); tf=tf+1; end

    // ===== WRONG PARITY (inject error during parity bit) =====
    // The TX generates correct parity, but we flip the parity bit on the loopback
    // This is the most realistic test: hardware generates correct parity,
    // but the line is corrupted.

    // TEST 4: 0x55 EVEN, inject parity error -> pe=1
    send_byte(8'h55, 3'd3, PARITY_EVEN, 1);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && parity_error) begin $display("  TEST 4 PASS (0x55 EVEN inject)"); tp=tp+1; end
    else begin $display("  TEST 4 FAIL pe=%b", parity_error); tf=tf+1; end

    // TEST 5: 0x55 ODD, inject parity error -> pe=1
    send_byte(8'h55, 3'd3, PARITY_ODD, 1);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && parity_error) begin $display("  TEST 5 PASS (0x55 ODD inject)"); tp=tp+1; end
    else begin $display("  TEST 5 FAIL pe=%b", parity_error); tf=tf+1; end

    // TEST 6: 0xAA EVEN, inject parity error -> pe=1
    send_byte(8'hAA, 3'd3, PARITY_EVEN, 1);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && parity_error) begin $display("  TEST 6 PASS (0xAA EVEN inject)"); tp=tp+1; end
    else begin $display("  TEST 6 FAIL pe=%b", parity_error); tf=tf+1; end

    // TEST 7: 0xAA ODD, inject parity error -> pe=1
    send_byte(8'hAA, 3'd3, PARITY_ODD, 1);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && parity_error) begin $display("  TEST 7 PASS (0xAA ODD inject)"); tp=tp+1; end
    else begin $display("  TEST 7 FAIL pe=%b", parity_error); tf=tf+1; end

    // TEST 8: 0x01 EVEN, inject parity error -> pe=1
    send_byte(8'h01, 3'd3, PARITY_EVEN, 1);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && parity_error) begin $display("  TEST 8 PASS (0x01 EVEN inject)"); tp=tp+1; end
    else begin $display("  TEST 8 FAIL pe=%b", parity_error); tf=tf+1; end

    // TEST 9: 0x01 ODD, inject parity error -> pe=1
    send_byte(8'h01, 3'd3, PARITY_ODD, 1);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && parity_error) begin $display("  TEST 9 PASS (0x01 ODD inject)"); tp=tp+1; end
    else begin $display("  TEST 9 FAIL pe=%b", parity_error); tf=tf+1; end

    // ===== NO PARITY mode (injection should have no effect) =====
    // TEST 10: 0x55 NONE, inject has no effect -> pe=0
    send_byte(8'h55, 3'd3, PARITY_NONE, 1);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && !parity_error) begin $display("  TEST 10 PASS (NONE inject, pe=0)"); tp=tp+1; end
    else begin $display("  TEST 10 FAIL pe=%b", parity_error); tf=tf+1; end

    // ===== Edge cases: 5-bit and 6-bit data =====
    // TEST 11: 5-bit 0x03 EVEN, inject -> pe=1
    send_byte(8'h03, 3'd0, PARITY_EVEN, 1);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && parity_error) begin $display("  TEST 11 PASS (5-bit inject)"); tp=tp+1; end
    else begin $display("  TEST 11 FAIL pe=%b", parity_error); tf=tf+1; end

    // TEST 12: 6-bit 0x2A ODD, no injection -> pe=0
    send_byte(8'h2A, 3'd1, PARITY_ODD, 0);
    timeout = 50000; waited = 0;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) waited = 1;
    if (waited && !parity_error) begin $display("  TEST 12 PASS (6-bit no inject)"); tp=tp+1; end
    else begin $display("  TEST 12 FAIL pe=%b", parity_error); tf=tf+1; end

    $display("");
    $display("=== RESULTS: %0d pass, %0d fail (12 tests) ===", tp, tf);
    $display("Waveform: tb_uart_parity_loopback.vcd");
    #5000; $finish;
  end

  initial begin
    #200_000_000;  // 200ms timeout
    $display("TIMEOUT");
    $finish;
  end

endmodule
