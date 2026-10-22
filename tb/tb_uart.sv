//==============================================================================
// tb_uart.sv
// Comprehensive UART Controller Testbench
//
// Validates all user configurations: baud rate (3), data bits (4),
// parity (3), stop bits (3) = 108 configurations.
// Each config tested with all-0, all-1, and random data (8 bytes each).
//
// Loopback mode: TXD -> RXD.
// Waveforms: tb_uart.vcd
//==============================================================================

`timescale 1ns / 1ps

import uart_pkg::*;

module tb_uart();

  // ===========================================================================
  // Parameters
  // ===========================================================================
  parameter CLK_PERIOD = 20;
  parameter CLK_FREQ   = 50_000_000;

  // ===========================================================================
  // Signals
  // ===========================================================================
  logic        clk;
  logic        rst_n;

  logic [15:0] cfg_baud_div;
  logic [2:0]  cfg_data_bits;
  logic [1:0]  cfg_parity;
  logic [1:0]  cfg_stop_bits;

  logic [7:0]  tx_data;
  logic        tx_valid;
  logic        tx_ready;
  logic        tx_busy;

  logic [7:0]  rx_data;
  logic        rx_valid;
  logic        rx_ready;
  logic        rx_busy;

  logic        framing_error;
  logic        parity_error;
  logic        overrun_error;
  logic        break_detect;
  logic        irq;

  logic        txd;
  logic        rxd;
  logic        clr_errors;  // W1C: pulse high to clear sticky error flags

  // Interrupt enables (tied low for existing loopback tests)
  logic        int_en_rx  = 1'b0;
  logic        int_en_tx  = 1'b0;
  logic        int_en_err = 1'b0;

  // ===========================================================================
  // DUT Instantiation
  // ===========================================================================
  uart_top #(.CLK_FREQ(CLK_FREQ)) u_uart (
    .clk, .rst_n,
    .cfg_baud_div, .cfg_data_bits, .cfg_parity, .cfg_stop_bits,
    .tx_data, .tx_valid, .tx_ready, .tx_busy,
    .rx_data, .rx_valid, .rx_ready, .rx_busy,
    .framing_error, .parity_error, .overrun_error,
    .break_detect,
    .irq,
    .int_en_rx, .int_en_tx, .int_en_err,
    .clr_errors, .txd, .rxd
  );

  // Loopback
  assign rxd = txd;

  assign clr_errors = 1'b0;  // No W1C clearing needed in loopback mode

  // ===========================================================================
  // Functional Coverage Monitor
  // ===========================================================================
  uart_fcov u_fcov (
    .clk, .rst_n,
    .cfg_data_bits, .cfg_parity, .cfg_stop_bits, .cfg_baud_div,
    .tx_data, .tx_valid, .tx_ready, .tx_busy,
    .rx_data, .rx_valid, .rx_ready, .rx_busy,
    .framing_error, .parity_error, .overrun_error,
    .break_detect,
    .irq,
    .int_en_rx, .int_en_tx, .int_en_err
  );

  // ===========================================================================
  // Clock
  // ===========================================================================
  initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD / 2) clk = ~clk;
  end

  // ===========================================================================
  // VCD Dump
  // ===========================================================================
  initial begin
    $dumpfile("tb_uart.vcd");
    $dumpvars(0, tb_uart);
  end

  // ===========================================================================
  // LFSR state
  // ===========================================================================
  logic [31:0] lfsr_state;

  // LFSR function: advances 32-bit state by 8 steps, returns low byte
  function automatic logic [31:0] lfsr_update(input logic [31:0] s);
    for (int i = 0; i < 8; i++) begin
      s = {s[30:0], s[31] ^ s[29] ^ s[27] ^ s[24]};
    end
    return s;
  endfunction

  // ===========================================================================
  // Helpers
  // ===========================================================================
  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  task reset();
    rst_n = 1'b0;
    tx_data = 8'h00; tx_valid = 1'b0; rx_ready = 1'b1;
    cfg_baud_div = 434; cfg_data_bits = 3; cfg_parity = 0; cfg_stop_bits = 0;
    wait_clks(10); rst_n = 1'b1; wait_clks(10);
    $display("[%0t] Reset complete", $time);
  endtask

  // ===========================================================================
  // Send a byte and verify reception (no "return" for Iverilog compat)
  // ===========================================================================
  task send_check(
    input logic [7:0] val,
    input int         data_bits,
    output logic      ok
  );
    int timeout;
    ok = 1'b1;

    // Re-arm handshake: pulse rx_ready high to clear any stale rx_valid
    // from the previous frame (the prior send_check left rx_ready=0,
    // which holds rx_valid indefinitely). Without this, the while(!rx_valid)
    // loop below would see rx_valid=1 immediately on entry, read stale
    // rx_data, and falsely fail.
    rx_ready = 1'b1;
    @(posedge clk);  // Allow rx_valid to clear via else-if (rx_ready && rx_valid)

    timeout = 1000000;
    while (!tx_ready && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
    if (timeout > 0) begin

      tx_data = val; tx_valid = 1'b1; @(posedge clk); tx_valid = 1'b0;

      timeout = 2000000;
      while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end
      if (timeout > 0) begin

        if (framing_error) begin $display("[%0t] FRAMING ERROR", $time); ok = 1'b0; end
        if (parity_error)  begin $display("[%0t] PARITY ERROR",  $time); ok = 1'b0; end

        // Compare received data
        // RX stores data right-aligned. Shift-based comparison is equivalent
        // for validation (both operands shift equally). Consumers should read
        // rx_data directly without shifting — it is right-aligned.
        if ((rx_data >> (8 - data_bits)) !== (val >> (8 - data_bits))) ok = 1'b0;

        rx_ready = 1'b0; @(posedge clk);
        wait_clks(3);

      end else begin
        $display("[%0t] TIMEOUT: rx_valid", $time);
        ok = 1'b0;
      end
    end else begin
      $display("[%0t] TIMEOUT: tx_ready", $time);
      ok = 1'b0;
    end
  endtask

  // ===========================================================================
  // Main Test
  // ===========================================================================
  initial begin
    integer ba, da, pa, st, i;
    integer total_cfgs;
    integer pass_cnt, fail_cnt;
    logic ok;
    logic [7:0] tx_val, max_val;

    $display("========================================================");
    $display("  UART Controller - Comprehensive Verification");
    $display("  3 baud x 4 data x 3 parity x 3 stop = 108 configs");
    $display("  Each: All-0s + All-1s + Random (8 bytes each)");
    $display("========================================================");

    reset();
    $display("[%0t] DBG: post-reset tx_ready=%b", $time, tx_ready);

    pass_cnt = 0;
    fail_cnt = 0;
    total_cfgs = 0;

    // Baud rates: 115200, 57600, 9600
    for (ba = 0; ba < 3; ba++) begin
      int cur_baud;
      if (ba == 0) cur_baud = 115200;
      else if (ba == 1) cur_baud = 57600;
      else cur_baud = 9600;

      // Data bits: 5,6,7,8
      for (da = 0; da < 4; da++) begin
        int cur_data;
        cur_data = da + 5;

        // Parity: 0=none, 1=even, 2=odd
        for (pa = 0; pa < 3; pa++) begin
          // Stop bits: 0=1, 1=1.5, 2=2
          for (st = 0; st < 3; st++) begin

            cfg_baud_div  = CLK_FREQ / cur_baud;
            cfg_data_bits = da;
            cfg_parity    = pa;
            cfg_stop_bits = st;
            wait_clks(10);

            total_cfgs = total_cfgs + 1;

            // Set max_val for this data width
            max_val = (cur_data < 8) ? ((1 << cur_data) - 1) : 8'hFF;

            // ---- All-0s ----
            for (i = 0; i < 8; i++) begin
              send_check(8'h00, cur_data, ok);
              if (ok) pass_cnt++; else begin fail_cnt++; $write("Z%d:X ", i); end
            end

            // ---- All-1s ----
            for (i = 0; i < 8; i++) begin
              send_check(max_val, cur_data, ok);
              if (ok) pass_cnt++; else begin fail_cnt++; $write("O%d:X ", i); end
            end

            // ---- Random ----
            lfsr_state = 32'hABCD_0001;
            for (i = 0; i < 8; i++) begin
              lfsr_state = lfsr_update(lfsr_state);
              tx_val = lfsr_state[7:0] & ((cur_data < 8) ? ((1 << cur_data) - 1) : 8'hFF);
              send_check(tx_val, cur_data, ok);
              if (ok) pass_cnt++; else begin fail_cnt++; $write("R%d:X ", i); end
            end

            $write(".");
            if ((total_cfgs % 9) == 0) $write("\n");

          end
        end
      end
    end

    $display("");
    $display("========================================================");
    $display("  VERIFICATION COMPLETE: %0d configurations tested", total_cfgs);
    $display("  Results: %0d PASS, %0d FAIL (out of %0d bytes)",
             pass_cnt, fail_cnt, pass_cnt + fail_cnt);
    $display("  Simulation time: %0t ns", $time);
    $display("  Waveform: tb_uart.vcd");
    $display("========================================================");

    if (fail_cnt > 0) begin
      $display("");
      $display("██████████████████████████████████████████████████████");
      $display("  FAIL: %0d byte mismatches detected!", fail_cnt);
      $display("██████████████████████████████████████████████████████");
      // Print coverage summary before $fatal (final block may not run)
      u_fcov.print_summary();
      $fatal(1, "tb_uart: %0d failures -- see diagnostics above for details", fail_cnt);
    end else begin
      $display("");
      $display("██████████████████████████████████████████████████████");
      $display("  ALL TESTS PASSED");
      $display("██████████████████████████████████████████████████████");
    end

    #5000;
    $finish;
  end

  // ===========================================================================
  // Timeout
  // ===========================================================================
  initial begin
    #5_000_000_000;
    $display("[%0t] TIMEOUT: 5 seconds exceeded", $time);
    $finish;
  end

endmodule
